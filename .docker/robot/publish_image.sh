#!/usr/bin/env bash
# Publishes the robot images that passed CI's smoke test: the publish job in
# .github/workflows/robot_container.yml runs it on noetic-devel. It doesn't run the images. It reads
# the archives that the two build jobs saved, checks their platform and commit, copies them to the
# registry, and combines them under one name for both architectures:
#   <image>:noetic-<commit>-amd64, -arm64   the image of each architecture
#   <image>:noetic-<commit>-index           the combined index before it is checked
#   <image>:noetic-<commit>                 both: published once per commit, a rerun reuses it
#   <image>:noetic                          the newest: it moves only forward in git history
# Usage, in a git checkout with the whole history, with skopeo, jq and docker buildx:
#   publish_image.sh <image> <commit> <archive folder>
# The archive folder holds robot-image-amd64/robot-image.tar.gz and robot-image-arm64/robot-image.tar.gz.
# FIRST_PUBLICATION=true allows the first publication of a package that the registry won't let
# this job read before it exists (step 2). INSECURE_REGISTRY=true is for a local test registry.
#
# Steps:
#   1. Check both archives: each holds the image for its architecture, built from this commit.
#      Their description label, which the Dockerfile sets, becomes the index's description.
#   2. Ask the registry whether the version and noetic exist. Only a first publication may
#      create the package first, under the tag first-publication.
#   3. Publish the version, unless it exists: copy both images, combine them under the tag
#      <version>-index, check that index, and then give its digest the version's tag.
#   4. Check the version, the one just published or the one that existed.
#   5. Move noetic to the version if noetic is missing or at an earlier commit.
#   6. Write what was published to the job summary.
set -euo pipefail

readonly REVISION=org.opencontainers.image.revision
readonly DESCRIPTION_KEY=org.opencontainers.image.description
readonly INDEX_TYPE=application/vnd.oci.image.index.v1+json
# A read that gives neither present nor missing is tried this often, with this pause between
readonly READ_TRIES=3
readonly READ_PAUSE=15 # s

# Set by main from the arguments: the image, the commit, the archive folder and the version
image='' commit='' archives='' version=''
# A temporary folder for the digests of the copies, removed at the exit
work=''
# The options of skopeo's writes and reads (registry_options)
copy_options=()
read_options=()
# The images' description, from their label (step 1)
description=''
# What the registry said about the version and noetic (step 2)
version_state='' noetic_state=''
# The checked index of the version, and its digest (step 4)
index_raw='' index=''

usage() {
  echo "Usage: $0 <image> <commit> <archive folder>"
}

# skopeo copies each archive with OCI manifests: only then can the combined index carry
# annotations. Docker's own push from its classic image store would send Docker's format.
# skopeo also asks the registry whether a tag exists (state), with the same login as the copies.
registry_options() {
  local auth_file=${DOCKER_CONFIG:-$HOME/.docker}/config.json
  copy_options=(--format oci)
  read_options=()
  if [ "${INSECURE_REGISTRY:-}" = true ]; then
    copy_options+=(--dest-tls-verify=false)
    read_options+=(--tls-verify=false)
  fi
  if [ -f "$auth_file" ]; then
    copy_options+=(--dest-authfile "$auth_file")
    read_options+=(--authfile "$auth_file")
  fi
}

fail() {
  echo "$*" >&2
  exit 1
}

archive_of() {
  echo "$archives/robot-image-$1/robot-image.tar.gz"
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

# Step 1. Each archive must hold the image for its architecture, built from this commit, with the
# description of the other archive. The first archive's description is kept in description.
check_archive() {
  local config found label
  config=$(skopeo inspect --config "docker-archive:$(archive_of "$1")")
  found=$(jq -r --arg label "$REVISION" '"\(.os)/\(.architecture) \(.config.Labels[$label] // "")"' <<<"$config")
  if [ "$found" != "linux/$1 $commit" ]; then
    fail "robot-image-$1 holds $found, not linux/$1 $commit"
  fi
  label=$(jq -r --arg key "$DESCRIPTION_KEY" '.config.Labels[$key] // ""' <<<"$config")
  if [ -z "$label" ]; then
    fail "robot-image-$1 has no description label"
  elif [ -z "$description" ]; then
    description=$label
  elif [ "$label" != "$description" ]; then
    fail "robot-image-$1 has another description than the other archive: $label"
  fi
}

# Step 2. What the registry says about a tag, <image>:<tag>:
#   present      The registry returned the tag's manifest.
#   missing      The registry answered the request for this tag's manifest with its error code
#                MANIFEST_UNKNOWN, which skopeo prints as "manifest unknown". Only this answer
#                counts. Other failures can read alike: a login helper that isn't installed and a
#                token service that answers 404 both print "not found", and neither says anything
#                about the tag.
#   unreadable   The registry itself refuses: it denies the request for this tag's manifest
#                (DENIED, UNAUTHORIZED), it doesn't know the repository (NAME_UNKNOWN), or its
#                token service answers 403. Each means that this job can't see the package,
#                which only the first publication may answer with a write (see below). The
#                refusal and the tag go to stderr, so that the job's log says which one it was.
# Each answer is recognized by the text that skopeo prints for it, with this tag and repository
# in it. A text that skopeo words differently one day is none of them, and the job stops.
# A read that gives neither present nor missing is tried READ_TRIES times. After that, an error
# that is not unreadable stops the job: a failed login, a file that can't be read, an error of
# the token service or of the connection. The version and noetic are written only after both
# read as present or missing.
state() {
  local reference=$1 error="" try refusal
  local repository=${reference%:*} tag=${reference##*:}
  local manifest_request="reading manifest $tag in $repository"
  # Each refusal as skopeo prints it, then what it means, separated by a |
  local refusals=(
    "$manifest_request: denied|the registry denies the read"
    "$manifest_request: unauthorized|the registry wants another login"
    "$manifest_request: name unknown|the registry doesn't know the repository"
    "Requesting bearer token: invalid status code from registry 403 (Forbidden)|the token service refuses a token (403)"
  )
  for ((try = 1; try <= READ_TRIES; try++)); do
    if [ "$try" -gt 1 ]; then
      sleep "$READ_PAUSE"
    fi
    if error=$(skopeo inspect --raw "${read_options[@]}" "docker://$reference" 2>&1 >/dev/null); then
      echo present
      return
    fi
    if grep -q -F "$manifest_request: manifest unknown" <<<"$error"; then
      echo missing
      return
    fi
  done
  for refusal in "${refusals[@]}"; do
    if grep -q -F "${refusal%%|*}" <<<"$error"; then
      echo "$reference: ${refusal#*|}" >&2
      echo unreadable
      return
    fi
  done
  echo "$error" >&2
  exit 1
}

# Step 2, the reads of both tags. A package that doesn't exist yet may be unreadable instead of
# missing. Only a run with FIRST_PUBLICATION=true, with both tags unreadable, creates the package:
# under the tag first-publication, which nothing else uses, from the amd64 archive that step 1
# checked. Then both tags must read as present or missing.
read_tags() {
  version_state=$(state "$version")
  noetic_state=$(state "$image:noetic")
  if [ "$version_state" != unreadable ] && [ "$noetic_state" != unreadable ]; then
    return
  fi
  if [ "${FIRST_PUBLICATION:-}" = true ] && [ "$version_state" = unreadable ] &&
    [ "$noetic_state" = unreadable ]; then
    echo "First publication: the registry doesn't let this job read $image. Creating it."
    skopeo copy "${copy_options[@]}" "docker-archive:$(archive_of amd64)" \
      "docker://$image:first-publication"
    version_state=$(state "$version")
    noetic_state=$(state "$image:noetic")
  fi
  if [ "$version_state" = unreadable ] || [ "$noetic_state" = unreadable ]; then
    fail "The registry doesn't let this job read $version or $image:noetic"
  fi
}

# Steps 3 and 4. An index of the version must be an OCI index for this commit, with the images'
# description and exactly one image per architecture, each built for its platform from this
# commit. Keeps the checked index in index_raw.
check_index() {
  local reference=$1 raw found image_digest platform
  raw=$(docker buildx imagetools inspect "$reference" --raw)
  found=$(jq -r --arg label "$REVISION" '"\(.mediaType) \(.annotations[$label] // "")"' <<<"$raw")
  if [ "$found" != "$INDEX_TYPE $commit" ]; then
    fail "$reference is $found, not an OCI index for $commit"
  fi
  found=$(jq -r --arg key "$DESCRIPTION_KEY" '.annotations[$key] // ""' <<<"$raw")
  if [ "$found" != "$description" ]; then
    fail "$reference lacks the images' description"
  fi
  found=$(images_of "$raw" | cut -d' ' -f2 | sort | paste -sd' ')
  if [ "$found" != "linux/amd64 linux/arm64" ]; then
    fail "$reference has the images $found, not linux/amd64 linux/arm64"
  fi
  while read -r image_digest platform; do
    found=$(docker buildx imagetools inspect "$image@$image_digest" --format '{{json .Image}}' |
      jq -r --arg label "$REVISION" '"\(.os)/\(.architecture) \(.config.Labels[$label] // "")"')
    if [ "$found" != "$platform $commit" ]; then
      fail "$reference's $platform entry holds an image for $found, not $platform $commit"
    fi
  done < <(images_of "$raw")
  index_raw=$raw
}

# Step 3. The version's tag goes only to an index that passed check_index
publish_version() {
  local arch candidate sources=()
  for arch in amd64 arm64; do
    skopeo copy "${copy_options[@]}" --digestfile "$work/$arch" \
      "docker-archive:$(archive_of "$arch")" "docker://$version-$arch"
    sources+=("$image@$(cat "$work/$arch")")
  done
  docker buildx imagetools create --tag "$version-index" \
    --annotation "index:$DESCRIPTION_KEY=$description" \
    --annotation "index:$REVISION=$commit" \
    "${sources[@]}"
  candidate=$(digest_of "$version-index")
  check_index "$image@$candidate"
  docker buildx imagetools create --tag "$version" "$image@$candidate"
  if [ "$(digest_of "$version")" != "$candidate" ]; then
    fail "$version doesn't point to the checked index $candidate"
  fi
}

promote() {
  docker buildx imagetools create --tag "$image:noetic" "$image@$index"
  if [ "$(digest_of "$image:noetic")" != "$index" ]; then
    fail "$image:noetic doesn't point to $index"
  fi
  echo "$image:noetic is now $version"
}

# Step 5. Move noetic to this version only forward: from the commit it has now to a later one.
# Its commit has to be a full commit ID of this repository's history, or noetic stays.
move_noetic() {
  local current
  if [ "$noetic_state" = missing ]; then
    promote
    return
  fi
  current=$(docker buildx imagetools inspect "$image:noetic" --raw |
    jq -r --arg label "$REVISION" '.annotations[$label] // ""')
  if ! [[ $current =~ ^[0-9a-f]{40}$ ]] ||
    ! git rev-parse --quiet --verify "$current^{commit}" > /dev/null; then
    fail "$image:noetic has the commit annotation '$current', which isn't a commit of this repository"
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
}

# Step 6. What was published, from the index that step 4 checked
write_summary() {
  local image_digest platform
  {
    echo "| Published | |"
    echo "|:--|:--|"
    echo "| Version | \`$version\` |"
    echo "| Index | \`$image@$index\` |"
    while read -r image_digest platform; do
      echo "| $platform | \`$image_digest\` |"
    done < <(images_of "$index_raw")
    echo "| Commit | \`$commit\` |"
    if [ -n "${GITHUB_RUN_ID:-}" ]; then
      echo "| Run | $GITHUB_SERVER_URL/$GITHUB_REPOSITORY/actions/runs/$GITHUB_RUN_ID |"
    fi
  } | tee -a "${GITHUB_STEP_SUMMARY:-/dev/null}"
}

main() {
  if [ "$#" -ne 3 ]; then
    usage >&2
    exit 2
  fi
  image=$1 commit=$2 archives=$3
  version=$image:noetic-$commit
  work=$(mktemp -d)
  trap 'rm -rf "$work"' EXIT
  registry_options
  # 1. Both archives before any write, also before a first publication
  check_archive amd64
  check_archive arm64
  # 2.
  read_tags
  # 3. and 4.
  if [ "$version_state" = present ]; then
    echo "$version is published already: reusing it, unchanged"
    check_index "$version"
  else
    publish_version
  fi
  index=$(digest_of "$version")
  # 5.
  move_noetic
  # 6.
  write_summary
}

main "$@"
