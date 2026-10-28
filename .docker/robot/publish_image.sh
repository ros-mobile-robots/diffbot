#!/usr/bin/env bash
# Publishes the robot images that passed CI's smoke test: the publish job in
# .github/workflows/robot_container.yml runs it on noetic-devel. It doesn't run the images. It reads
# the archives that the two build jobs saved, checks their platform and commit, copies them to the
# registry, and combines them under one name for both architectures:
#   <image>:noetic-<commit>-amd64, -arm64   the image of each architecture
#   <image>:noetic-<commit>                 both: published once per commit, a rerun reuses it
#   <image>:noetic                          the newest: it moves only forward in git history
# Usage, in a git checkout with the whole history, with skopeo, jq and docker buildx:
#   publish_image.sh <image> <commit> <archive folder>
# The archive folder holds robot-image-amd64/robot-image.tar.gz and robot-image-arm64/robot-image.tar.gz.
# FIRST_PUBLICATION=true allows the first publication of a package that the registry won't let
# this job read before it exists (see below). INSECURE_REGISTRY=true is for a local test registry.
set -euo pipefail

image=$1 commit=$2 archives=$3
version=$image:noetic-$commit
revision=org.opencontainers.image.revision
description="ROS Noetic and the robot's packages for DiffBot and Remo, for the robot's Raspberry Pi"

# skopeo copies each archive with OCI manifests: only then can the combined index carry
# annotations. Docker's own push from its classic image store would send Docker's format.
copy_options=(--format oci)
if [ "${INSECURE_REGISTRY:-}" = true ]; then
  copy_options+=(--dest-tls-verify=false)
fi
auth_file=${DOCKER_CONFIG:-$HOME/.docker}/config.json
if [ -f "$auth_file" ]; then
  copy_options+=(--dest-authfile "$auth_file")
fi

# What the registry says about a tag: "present", "missing" if it says the tag doesn't exist, or
# "unreadable" if it denies reading it. Each check is tried three times, and other errors stop the
# job. The version and noetic are written only after both read as present or missing.
state() {
  local err=""
  for try in 1 2 3; do
    if [ "$try" -gt 1 ]; then
      sleep 15
    fi
    if err=$(docker buildx imagetools inspect "$1" 2>&1 >/dev/null); then
      echo present
      return
    fi
    if grep -q -i 'not found' <<<"$err"; then
      echo missing
      return
    fi
  done
  if grep -q -i -E 'denied|unauthorized|forbidden' <<<"$err"; then
    echo unreadable
    return
  fi
  echo "$err" >&2
  exit 1
}

fail() {
  echo "$*" >&2
  exit 1
}

digest_of() {
  docker buildx imagetools inspect "$1" --format '{{json .Manifest}}' | jq -r .digest
}

# The images in an index, without attestations: one "<digest> <os>/<architecture>" per line
images_of() {
  jq -r '.manifests[]
    | select(.annotations["vnd.docker.reference.type"] != "attestation-manifest")
    | "\(.digest) \(.platform.os)/\(.platform.architecture)"' <<<"$1"
}

# A published version must be an OCI index for this commit with exactly one image per
# architecture, each built for its platform from this commit
check_version() {
  local raw found image_digest platform
  raw=$(docker buildx imagetools inspect "$version" --raw)
  found=$(jq -r --arg label "$revision" '"\(.mediaType) \(.annotations[$label] // "")"' <<<"$raw")
  if [ "$found" != "application/vnd.oci.image.index.v1+json $commit" ]; then
    fail "$version is $found, not an OCI index for $commit"
  fi
  if [ "$(jq -r '.annotations["org.opencontainers.image.description"] // ""' <<<"$raw")" != "$description" ]; then
    fail "$version lacks the description"
  fi
  found=$(images_of "$raw" | cut -d' ' -f2 | sort | paste -sd' ')
  if [ "$found" != "linux/amd64 linux/arm64" ]; then
    fail "$version has the images $found, not linux/amd64 linux/arm64"
  fi
  while read -r image_digest platform; do
    found=$(docker buildx imagetools inspect "$image@$image_digest" --format '{{json .Image}}' |
      jq -r --arg label "$revision" '"\(.os)/\(.architecture) \(.config.Labels[$label] // "")"')
    if [ "$found" != "$platform $commit" ]; then
      fail "$version's $platform entry holds an image for $found, not $platform $commit"
    fi
  done < <(images_of "$raw")
}

# Each archive must hold the image for its architecture, built from this commit
check_archive() {
  local found
  found=$(skopeo inspect --config "docker-archive:$archives/robot-image-$1/robot-image.tar.gz" |
    jq -r --arg label "$revision" '"\(.os)/\(.architecture) \(.config.Labels[$label] // "")"')
  if [ "$found" != "linux/$1 $commit" ]; then
    fail "robot-image-$1 holds $found, not linux/$1 $commit"
  fi
}

version_state=$(state "$version")
noetic_state=$(state "$image:noetic")

# A package that doesn't exist yet may be unreadable instead of missing. Only a run with
# FIRST_PUBLICATION=true, with both tags unreadable, creates the package: under the tag
# first-publication, which nothing else uses. Then both tags must read as present or missing.
if [ "$version_state" = unreadable ] || [ "$noetic_state" = unreadable ]; then
  if [ "${FIRST_PUBLICATION:-}" = true ] && [ "$version_state" = unreadable ] &&
    [ "$noetic_state" = unreadable ]; then
    echo "First publication: the registry doesn't let this job read $image. Creating it."
    check_archive amd64
    skopeo copy "${copy_options[@]}" \
      "docker-archive:$archives/robot-image-amd64/robot-image.tar.gz" "docker://$image:first-publication"
    version_state=$(state "$version")
    noetic_state=$(state "$image:noetic")
  fi
  if [ "$version_state" = unreadable ] || [ "$noetic_state" = unreadable ]; then
    fail "The registry doesn't let this job read $version or $image:noetic"
  fi
fi

if [ "$version_state" = present ]; then
  echo "$version is published already: reusing it, unchanged"
else
  # Check both archives before copying either
  check_archive amd64
  check_archive arm64
  digests=$(mktemp -d)
  sources=()
  for arch in amd64 arm64; do
    skopeo copy "${copy_options[@]}" --digestfile "$digests/$arch" \
      "docker-archive:$archives/robot-image-$arch/robot-image.tar.gz" "docker://$version-$arch"
    sources+=("$image@$(cat "$digests/$arch")")
  done
  docker buildx imagetools create --tag "$version" \
    --annotation "index:org.opencontainers.image.description=$description" \
    --annotation "index:$revision=$commit" \
    "${sources[@]}"
fi
check_version
index=$(digest_of "$version")

# Move noetic to this version only forward: from the commit it has now to a later one
promote() {
  docker buildx imagetools create --tag "$image:noetic" "$image@$index"
  if [ "$(digest_of "$image:noetic")" != "$index" ]; then
    fail "$image:noetic doesn't point to $index"
  fi
  echo "$image:noetic is now $version"
}
if [ "$noetic_state" = present ]; then
  current=$(docker buildx imagetools inspect "$image:noetic" --raw |
    jq -r --arg label "$revision" '.annotations[$label] // ""')
  if [ -z "$current" ]; then
    fail "$image:noetic has no commit annotation"
  elif [ "$current" = "$commit" ]; then
    if [ "$(digest_of "$image:noetic")" != "$index" ]; then
      fail "$image:noetic has the commit $commit, but not the digest $index"
    fi
    echo "$image:noetic is $version already"
  elif git merge-base --is-ancestor "$current" "$commit"; then
    promote
  elif git merge-base --is-ancestor "$commit" "$current"; then
    echo "$image:noetic stays at the later commit $current"
  else
    fail "$image:noetic has the commit $current, which isn't before or after $commit in git history"
  fi
else
  promote
fi

# What was published, for the job summary
{
  echo "| Published | |"
  echo "|:--|:--|"
  echo "| Version | \`$version\` |"
  echo "| Index | \`$image@$index\` |"
  images_of "$(docker buildx imagetools inspect "$version" --raw)" |
    while read -r image_digest platform; do echo "| $platform | \`$image_digest\` |"; done
  echo "| Commit | \`$commit\` |"
  if [ -n "${GITHUB_RUN_ID:-}" ]; then
    echo "| Run | $GITHUB_SERVER_URL/$GITHUB_REPOSITORY/actions/runs/$GITHUB_RUN_ID |"
  fi
} | tee -a "${GITHUB_STEP_SUMMARY:-/dev/null}"
