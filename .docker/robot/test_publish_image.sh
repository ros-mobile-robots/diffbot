#!/usr/bin/env bash
# Tests which tags publish_image.sh writes for each answer of the registry, without a registry and
# without the images. Stand-ins take the place of skopeo, docker and sleep: they answer the
# script's reads with texts recorded from skopeo 1.13.3, and they note every write that the
# script attempts (skopeo copy, docker buildx imagetools create) instead of doing it. CI runs the
# tests in the workflow Robot container, before it builds the image
# (.github/workflows/robot_container.yml). To run them, with jq, in the repository's root folder:
#   .docker/robot/test_publish_image.sh
# The steps:
#   1. Write the stand-ins and a login file to a temporary folder, and put the stand-ins first
#      on PATH.
#   2. Run each test: it chooses the registry's answers, runs publish_image.sh, and compares the
#      script's exit code and its writes with the expected ones.
#   3. Print which tests failed, and exit with 1 if one did.
set -euo pipefail

readonly PUBLISHER="$PWD/.docker/robot/publish_image.sh"
# .invalid never resolves (RFC 2606), so a call that slips past the stand-ins reaches no registry
readonly IMAGE=registry.invalid/robot
readonly COMMIT=0123456789abcdef0123456789abcdef01234567
readonly VERSION=$IMAGE:noetic-$COMMIT
# The same text as in publish_image.sh: the test fails if the script annotates another one
readonly DESCRIPTION="ROS Noetic and the robot's packages for DiffBot and Remo, for the robot's Raspberry Pi"
readonly AMD64_DIGEST=sha256:1111111111111111111111111111111111111111111111111111111111111111
readonly ARM64_DIGEST=sha256:2222222222222222222222222222222222222222222222222222222222222222
readonly INDEX_DIGEST=sha256:3333333333333333333333333333333333333333333333333333333333333333
readonly OTHER_DIGEST=sha256:4444444444444444444444444444444444444444444444444444444444444444

# The writes of a complete publication, in the script's order, as the stand-ins note them: each
# copy with its archive and its tag, each create with every argument in brackets of its own, so
# that two arguments passed as one would show
readonly COMPLETE_PUBLICATION="copy robot-image-amd64 $VERSION-amd64
copy robot-image-arm64 $VERSION-arm64
create [--tag] [$VERSION] \
[--annotation] [index:org.opencontainers.image.description=$DESCRIPTION] \
[--annotation] [index:org.opencontainers.image.revision=$COMMIT] \
[$IMAGE@$AMD64_DIGEST] [$IMAGE@$ARM64_DIGEST]
create [--tag] [$IMAGE:noetic] [$IMAGE@$INDEX_DIGEST]"
readonly FIRST_PUBLICATION_COPY="copy robot-image-amd64 $IMAGE:first-publication"

work=$(mktemp -d)
readonly work
trap 'rm -rf "$work"' EXIT

# Step 1. The stand-ins read what to answer from the variables that run_publisher exports.
write_stand_ins() {
  mkdir "$work/bin"
  # The login file that docker login would write. publish_image.sh has to pass it to every
  # skopeo call that talks to the registry, and the skopeo stand-in refuses a call without it.
  echo '{"auths": {}}' > "$work/config.json"

  # skopeo: "inspect --raw docker://<tag>" asks the registry for a tag, "inspect --config" reads an
  # archive, "copy" writes to the registry
  cat > "$work/bin/skopeo" <<'STAND_IN'
#!/usr/bin/env bash
set -euo pipefail
last=${*: -1}
fatal() {
  echo "time=\"2026-11-18T10:38:22Z\" level=fatal msg=\"Error parsing image name \\\"$last\\\": $1\"" >&2
  exit 1
}
unexpected() {
  echo "unexpected: skopeo $*" >> "$WRITES"
  exit 90
}
# True if an option and its value are two arguments of their own, one after the other:
# has_option <option> <value> <the call's arguments...>
has_option() {
  local option=$1 value=$2
  shift 2
  while [ $# -gt 1 ]; do
    if [ "$1" = "$option" ] && [ "$2" = "$value" ]; then
      return 0
    fi
    shift
  done
  return 1
}
if [ "$1 $2" = "inspect --raw" ]; then
  has_option --authfile "$AUTH_FILE" "$@" || unexpected "$@"
  reference=${last#docker://}
  repository=${reference%:*} tag=${reference##*:}
  answer=$VERSION_ANSWER
  if [ "$tag" = noetic ]; then
    answer=$NOETIC_ANSWER
  fi
  case "$answer" in
    present) echo '{}' ;;
    manifest_unknown) fatal "reading manifest $tag in $repository: manifest unknown" ;;
    name_unknown) fatal "reading manifest $tag in $repository: name unknown" ;;
    denied) fatal "Requesting bearer token: invalid status code from registry 403 (Forbidden)" ;;
    token_service_not_found)
      fatal "Requesting bearer token: invalid status code from registry 404 (Not Found)" ;;
    page_not_found)
      fatal "reading manifest $tag in $repository: StatusCode: 404, <html><body>404 page not found</body></html>" ;;
    registry_unreachable)
      fatal "pinging container registry registry.invalid: Get \\\"https://registry.invalid/v2/\\\": dial tcp: lookup registry.invalid: no such host" ;;
    login_helper_missing)
      fatal "getting username and password: 1 error occurred:\\n\\t* error getting credentials - err: exec: \\\"docker-credential-unavailable-helper\\\": executable file not found in \$PATH, out: \`\`\\n\\n" ;;
    login_file_unreadable)
      fatal "getting username and password: 1 error occurred:\\n\\t* reading JSON file \\\"$AUTH_FILE\\\": open $AUTH_FILE: permission denied\\n\\n" ;;
  esac
elif [ "$1 $2" = "inspect --config" ]; then
  architecture=${last#*robot-image-}
  jq -n --arg architecture "${architecture%%/*}" --arg commit "$COMMIT" \
    '{os: "linux", architecture: $architecture,
      config: {Labels: {"org.opencontainers.image.revision": $commit}}}'
elif [ "$1" = copy ]; then
  has_option --format oci "$@" || unexpected "$@"
  has_option --dest-authfile "$AUTH_FILE" "$@" || unexpected "$@"
  archive=${*: -2:1}
  archive=${archive#*robot-image-}
  echo "copy robot-image-${archive%%/*} ${last#docker://}" >> "$WRITES"
  # The digest of the copied image, as the registry would report it for this architecture
  while [ $# -gt 0 ]; do
    if [ "$1" = --digestfile ]; then
      case "$last" in
        *-amd64) echo "$AMD64_DIGEST" > "$2" ;;
        *-arm64) echo "$ARM64_DIGEST" > "$2" ;;
      esac
    fi
    shift
  done
else
  unexpected "$@"
fi
STAND_IN

  # docker: "buildx imagetools inspect" reads a published tag, "buildx imagetools create" writes one
  cat > "$work/bin/docker" <<'STAND_IN'
#!/usr/bin/env bash
set -euo pipefail
revision=org.opencontainers.image.revision
if [ "$1 $2 $3" = "buildx imagetools create" ]; then
  shift 3
  { printf 'create'; printf ' [%s]' "$@"; printf '\n'; } >> "$WRITES"
elif [ "$1 $2 $3" != "buildx imagetools inspect" ]; then
  echo "unexpected: docker $*" >> "$WRITES"
  exit 90
elif [ "$5" = --raw ]; then
  jq -n --arg revision "$revision" --arg commit "$COMMIT" --arg description "$DESCRIPTION" \
    --arg amd64 "$AMD64_DIGEST" --arg arm64 "$ARM64_DIGEST" \
    '{mediaType: "application/vnd.oci.image.index.v1+json",
      annotations: {($revision): $commit, "org.opencontainers.image.description": $description},
      manifests: [
        {digest: $amd64, platform: {os: "linux", architecture: "amd64"}},
        {digest: $arm64, platform: {os: "linux", architecture: "arm64"}}]}'
elif [ "$6" = '{{json .Manifest}}' ]; then
  case "$4" in
    *:noetic) jq -n --arg digest "$NOETIC_DIGEST" '{digest: $digest}' ;;
    *) jq -n --arg digest "$INDEX_DIGEST" '{digest: $digest}' ;;
  esac
elif [ "$6" = '{{json .Image}}' ]; then
  case "$4" in
    *@"$AMD64_DIGEST") architecture=amd64 image_commit=$COMMIT ;;
    *) architecture=arm64 image_commit=$ARM64_IMAGE_COMMIT ;;
  esac
  jq -n --arg architecture "$architecture" --arg revision "$revision" --arg commit "$image_commit" \
    '{os: "linux", architecture: $architecture, config: {Labels: {($revision): $commit}}}'
fi
STAND_IN

  # sleep: the script waits 15 s between two tries of a read. The tests don't wait.
  printf '#!/usr/bin/env bash\n' > "$work/bin/sleep"

  chmod +x "$work/bin/skopeo" "$work/bin/docker" "$work/bin/sleep"
}

# Run publish_image.sh with the stand-ins. The first argument is the registry's answer to "does
# this tag exist?", for the version and for noetic. More arguments are variables for this run
# only, as NAME=value: NOETIC_ANSWER=... gives noetic another answer than the version.
# Sets exit_code and writes.
run_publisher() {
  local registry_answer=$1
  shift
  : > "$work/writes"
  exit_code=0
  env PATH="$work/bin:$PATH" DOCKER_CONFIG="$work" AUTH_FILE="$work/config.json" \
    GITHUB_STEP_SUMMARY=/dev/null WRITES="$work/writes" \
    VERSION_ANSWER="$registry_answer" NOETIC_ANSWER="$registry_answer" \
    COMMIT="$COMMIT" DESCRIPTION="$DESCRIPTION" AMD64_DIGEST="$AMD64_DIGEST" \
    ARM64_DIGEST="$ARM64_DIGEST" INDEX_DIGEST="$INDEX_DIGEST" NOETIC_DIGEST="$INDEX_DIGEST" \
    ARM64_IMAGE_COMMIT="$COMMIT" FIRST_PUBLICATION=false "$@" \
    "$PUBLISHER" "$IMAGE" "$COMMIT" "$work/archives" > "$work/output" 2>&1 || exit_code=$?
  writes=$(cat "$work/writes")
}

# Compare the last run with the expected exit code and the expected writes, one per line
assert_exit_code_and_writes() {
  local expected_exit_code=$1 expected_writes=$2
  if [ "$exit_code" -ne "$expected_exit_code" ]; then
    echo "  exit code $exit_code, expected $expected_exit_code"
    return 1
  fi
  if [ "$writes" != "$expected_writes" ]; then
    echo "  writes: '$writes', expected: '$expected_writes'"
    return 1
  fi
}

# The registry's own "manifest unknown" is the only answer that lets the script publish

test_publisher_publishes_both_images_and_both_tags_when_the_registry_reports_manifest_unknown() {
  run_publisher manifest_unknown
  assert_exit_code_and_writes 0 "$COMPLETE_PUBLICATION"
}

# Failures that also print "not found" or "denied" say nothing about the tag: no write, also not
# in a first publication

test_publisher_writes_nothing_when_the_login_helper_is_not_installed() {
  run_publisher login_helper_missing
  assert_exit_code_and_writes 1 ""
}

test_publisher_writes_nothing_when_the_token_service_answers_not_found() {
  run_publisher token_service_not_found
  assert_exit_code_and_writes 1 ""
}

test_publisher_writes_nothing_when_a_page_without_the_registry_error_code_answers_not_found() {
  run_publisher page_not_found
  assert_exit_code_and_writes 1 ""
}

test_publisher_writes_nothing_when_the_registry_cannot_be_reached() {
  run_publisher registry_unreachable
  assert_exit_code_and_writes 1 ""
}

test_first_publication_writes_nothing_when_the_login_file_cannot_be_read() {
  run_publisher login_file_unreadable FIRST_PUBLICATION=true
  assert_exit_code_and_writes 1 ""
}

# A registry that denies the read, or doesn't know the repository, stops a normal publication.
# A first publication answers with its own tag, and only if the registry refuses both reads.

test_publisher_writes_nothing_when_the_registry_denies_the_read() {
  run_publisher denied
  assert_exit_code_and_writes 1 ""
}

test_publisher_writes_nothing_when_the_registry_does_not_know_the_repository() {
  run_publisher name_unknown
  assert_exit_code_and_writes 1 ""
}

test_first_publication_writes_only_its_own_tag_when_the_registry_keeps_denying_the_read() {
  run_publisher denied FIRST_PUBLICATION=true
  assert_exit_code_and_writes 1 "$FIRST_PUBLICATION_COPY"
}

test_first_publication_writes_only_its_own_tag_when_the_repository_stays_unknown() {
  run_publisher name_unknown FIRST_PUBLICATION=true
  assert_exit_code_and_writes 1 "$FIRST_PUBLICATION_COPY"
}

test_first_publication_writes_nothing_when_the_registry_denies_only_one_of_the_two_reads() {
  run_publisher manifest_unknown NOETIC_ANSWER=denied FIRST_PUBLICATION=true
  assert_exit_code_and_writes 1 ""
}

# A version that is published already is checked and reused, never written again

test_publisher_writes_nothing_when_the_version_and_noetic_are_published() {
  run_publisher present
  assert_exit_code_and_writes 0 ""
}

test_publisher_writes_nothing_when_a_published_image_is_from_another_commit() {
  run_publisher present ARM64_IMAGE_COMMIT=another-commit
  assert_exit_code_and_writes 1 ""
}

test_publisher_writes_nothing_when_noetic_has_this_commit_but_another_digest() {
  run_publisher present NOETIC_DIGEST="$OTHER_DIGEST"
  assert_exit_code_and_writes 1 ""
}

readonly TESTS=(
  test_publisher_publishes_both_images_and_both_tags_when_the_registry_reports_manifest_unknown
  test_publisher_writes_nothing_when_the_login_helper_is_not_installed
  test_publisher_writes_nothing_when_the_token_service_answers_not_found
  test_publisher_writes_nothing_when_a_page_without_the_registry_error_code_answers_not_found
  test_publisher_writes_nothing_when_the_registry_cannot_be_reached
  test_first_publication_writes_nothing_when_the_login_file_cannot_be_read
  test_publisher_writes_nothing_when_the_registry_denies_the_read
  test_publisher_writes_nothing_when_the_registry_does_not_know_the_repository
  test_first_publication_writes_only_its_own_tag_when_the_registry_keeps_denying_the_read
  test_first_publication_writes_only_its_own_tag_when_the_repository_stays_unknown
  test_first_publication_writes_nothing_when_the_registry_denies_only_one_of_the_two_reads
  test_publisher_writes_nothing_when_the_version_and_noetic_are_published
  test_publisher_writes_nothing_when_a_published_image_is_from_another_commit
  test_publisher_writes_nothing_when_noetic_has_this_commit_but_another_digest
)

# Steps 2 and 3. A failed test prints the script's output, which says where the script stopped.
run_tests() {
  local failed=0 name
  for name in "${TESTS[@]}"; do
    if "$name"; then
      echo "ok      $name"
    else
      echo "FAILED  $name"
      sed 's/^/  | /' "$work/output"
      failed=$((failed + 1))
    fi
  done
  echo "${#TESTS[@]} tests, $failed failed"
  [ "$failed" -eq 0 ]
}

main() {
  write_stand_ins
  run_tests
}

main "$@"
