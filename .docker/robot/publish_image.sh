#!/usr/bin/env bash
# Publishes the robot images that passed CI's smoke test: the publish job in
# .github/workflows/robot_container.yml runs it on noetic-devel. It doesn't run the images. It loads
# the archives that the two build jobs saved, checks their platform and commit, pushes them, and
# combines them under one name for both architectures:
#   <image>:noetic-<commit>-amd64, -arm64   the image of each architecture
#   <image>:noetic-<commit>                 both: published once per commit, a rerun reuses it
#   <image>:noetic                          the newest: it moves only forward in git history
# Usage, in a git checkout with the whole history:
#   publish_image.sh <image> <commit> <archive folder>
# The archive folder holds robot-image-amd64/robot-image.tar.gz and robot-image-arm64/robot-image.tar.gz.
set -euo pipefail

image=$1 commit=$2 archives=$3
version=$image:noetic-$commit
revision=org.opencontainers.image.revision
description="ROS Noetic and the robot's packages for DiffBot and Remo, for the robot's Raspberry Pi"

# The digest of a tag in the registry
digest_of() {
  docker buildx imagetools inspect "$1" --format '{{json .Manifest}}' | jq -r .digest
}

# Whether a tag exists in the registry. A missing tag reads as "not found", or as "denied" while
# the package doesn't exist yet. A denial can't hide a version that this job could replace:
# pushing needs write access, which includes read access. Other errors stop the job.
exists() {
  local err
  if err=$(docker buildx imagetools inspect "$1" 2>&1 >/dev/null); then
    return 0
  fi
  if grep -q -i -E 'not found|denied|unauthorized|forbidden' <<<"$err"; then
    return 1
  fi
  echo "$err" >&2
  exit 1
}

# The platforms of an image in the registry with their commit, one "linux/amd64 <commit>" per line
platforms_of() {
  docker buildx imagetools inspect "$1" --format '{{json .Image}}' |
    jq -r --arg label "$revision" 'to_entries[] | "\(.key) \(.value.config.Labels[$label])"' |
    sort
}

# The published version must have exactly both architectures, built from this commit
check_version() {
  local found
  found=$(platforms_of "$version")
  if [ "$found" != "$(printf 'linux/amd64 %s\nlinux/arm64 %s' "$commit" "$commit")" ]; then
    printf '%s has other platforms or commits:\n%s\n' "$version" "$found" >&2
    exit 1
  fi
}

if exists "$version"; then
  echo "$version is published already: reusing it, unchanged"
  check_version
else
  # Check both images before pushing either
  for arch in amd64 arm64; do
    docker load --input "$archives/robot-image-$arch/robot-image.tar.gz"
    # Both archives hold diffbot-robot:noetic, so tag each image before loading the next
    found=$(docker image inspect diffbot-robot:noetic \
      --format "{{.Os}}/{{.Architecture}} {{index .Config.Labels \"$revision\"}}")
    if [ "$found" != "linux/$arch $commit" ]; then
      echo "robot-image-$arch holds $found, not linux/$arch $commit" >&2
      exit 1
    fi
    docker tag diffbot-robot:noetic "$version-$arch"
  done
  sources=()
  for arch in amd64 arm64; do
    docker push --quiet "$version-$arch"
    sources+=("$image@$(digest_of "$version-$arch")")
  done
  docker buildx imagetools create --tag "$version" \
    --annotation "index:org.opencontainers.image.description=$description" \
    --annotation "index:$revision=$commit" \
    "${sources[@]}"
  check_version
fi
index=$(digest_of "$version")

# Move noetic to this version only forward: from the commit it has now to a later one
promote() {
  docker buildx imagetools create --tag "$image:noetic" "$image@$index"
  if [ "$(digest_of "$image:noetic")" != "$index" ]; then
    echo "$image:noetic doesn't point to $index" >&2
    exit 1
  fi
  echo "$image:noetic is now $version"
}
current=""
if exists "$image:noetic"; then
  current=$(docker buildx imagetools inspect "$image:noetic" --format '{{json .Image}}' |
    jq -r --arg label "$revision" '.["linux/amd64"].config.Labels[$label] // empty')
fi
if [ -z "$current" ]; then
  promote
elif [ "$current" = "$commit" ]; then
  echo "$image:noetic is $version already"
elif git merge-base --is-ancestor "$current" "$commit"; then
  promote
elif git merge-base --is-ancestor "$commit" "$current"; then
  echo "$image:noetic stays at the later commit $current"
else
  echo "$image:noetic has the commit $current, which isn't before or after $commit in git history" >&2
  exit 1
fi

# What was published, for the job summary
{
  echo "| Published | |"
  echo "|:--|:--|"
  echo "| Version | \`$version\` |"
  echo "| Index | \`$image@$index\` |"
  docker buildx imagetools inspect "$version" --format '{{json .Manifest}}' |
    jq -r '.manifests[] | select(.platform.os == "linux") | "| \(.platform.architecture) | `\(.digest)` |"'
  echo "| Commit | \`$commit\` |"
  if [ -n "${GITHUB_RUN_ID:-}" ]; then
    echo "| Run | $GITHUB_SERVER_URL/$GITHUB_REPOSITORY/actions/runs/$GITHUB_RUN_ID |"
  fi
} | tee -a "${GITHUB_STEP_SUMMARY:-/dev/null}"
