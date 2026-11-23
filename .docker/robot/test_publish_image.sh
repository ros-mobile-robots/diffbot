#!/usr/bin/env bash
# Tests which tags publish_image.sh writes for each answer of the registry, without a registry and
# without the images. Stand-ins take the place of skopeo, docker and sleep: they answer the
# script's reads with texts recorded from skopeo 1.13.3, and they note every write that the
# script attempts (skopeo copy, docker buildx imagetools create) instead of doing it. git is the
# real one, in a small history of its own, so that "noetic only moves forward" is tested with
# git's own answers. CI runs the tests in the workflow Robot container, before it builds the image
# (.github/workflows/robot_container.yml). To run them, with jq and git, in the repository's root:
#   .docker/robot/test_publish_image.sh
# The steps:
#   1. Make the git history: EARLIER_COMMIT, then COMMIT, then LATER_COMMIT on one branch,
#      UNRELATED_COMMIT on a branch of its own, and an annotated tag of EARLIER_COMMIT. The
#      publisher publishes COMMIT, and HEAD is EARLIER_COMMIT.
#   2. Write the stand-ins and a login file to a temporary folder, and put the stand-ins first
#      on PATH.
#   3. Run each test: it chooses the registry's answers and the archives' content, runs
#      publish_image.sh, and compares the script's exit code and its writes with the expected ones.
#   4. Print which tests failed, and exit with 1 if one did.
set -euo pipefail

readonly PUBLISHER="$PWD/.docker/robot/publish_image.sh"
# .invalid never resolves (RFC 2606), so a call that slips past the stand-ins reaches no registry
readonly IMAGE=registry.invalid/robot
# The labels' and annotations' keys, as in publish_image.sh. The description is the one that the
# archives' images carry, which the script has to put on the index.
readonly REVISION=org.opencontainers.image.revision
readonly DESCRIPTION_KEY=org.opencontainers.image.description
readonly DESCRIPTION="The robot image of the test"
readonly AMD64_DIGEST=sha256:1111111111111111111111111111111111111111111111111111111111111111
readonly ARM64_DIGEST=sha256:2222222222222222222222222222222222222222222222222222222222222222
readonly INDEX_DIGEST=sha256:3333333333333333333333333333333333333333333333333333333333333333
readonly OTHER_DIGEST=sha256:4444444444444444444444444444444444444444444444444444444444444444
# 40 hexadecimal digits that are no commit of the test's history
readonly UNKNOWN_COMMIT=5555555555555555555555555555555555555555

work=$(mktemp -d)
readonly work
trap 'rm -rf "$work"' EXIT

# Set by step 1: the commits, the version, and the writes that the tests expect
EARLIER_COMMIT='' COMMIT='' LATER_COMMIT='' UNRELATED_COMMIT='' TAG_OBJECT='' VERSION=''
VERSION_PUBLICATION='' INDEX_CANDIDATE='' NOETIC_MOVE='' COMPLETE_PUBLICATION=''
FIRST_PUBLICATION_COPY=''

# git in the test's history, without the settings of the user or the system, so that no commit
# asks for a signature
history_git() {
  GIT_CONFIG_GLOBAL=/dev/null GIT_CONFIG_NOSYSTEM=1 git -C "$work/history" \
    -c user.name=test -c user.email=test@example.invalid "$@"
}

new_commit() {
  history_git commit -q --allow-empty -m "$1"
  history_git rev-parse HEAD
}

# Step 1. The commits, and the writes of the publications, as the stand-ins note them: each copy
# with its archive and its tag, each create with every argument in brackets of its own, so that
# two arguments passed as one would show
make_history() {
  mkdir "$work/history"
  history_git init -q
  EARLIER_COMMIT=$(new_commit earlier)
  COMMIT=$(new_commit published)
  LATER_COMMIT=$(new_commit later)
  history_git checkout -q --orphan unrelated
  UNRELATED_COMMIT=$(new_commit unrelated)
  # HEAD stays at EARLIER_COMMIT: a publisher that took a name such as HEAD from noetic's
  # annotation would read it as an earlier commit and move noetic
  history_git checkout -q --detach "$EARLIER_COMMIT"
  # An annotated tag of EARLIER_COMMIT: its own ID has 40 hexadecimal digits too, but it names a
  # tag object, not a commit
  history_git tag -a -m "a tag" earlier-tag "$EARLIER_COMMIT"
  TAG_OBJECT=$(history_git rev-parse earlier-tag)
  VERSION=$IMAGE:noetic-$COMMIT
  INDEX_CANDIDATE="copy robot-image-amd64 $VERSION-amd64
copy robot-image-arm64 $VERSION-arm64
create [--tag] [$VERSION-index] \
[--annotation] [index:$DESCRIPTION_KEY=$DESCRIPTION] [--annotation] [index:$REVISION=$COMMIT] \
[$IMAGE@$AMD64_DIGEST] [$IMAGE@$ARM64_DIGEST]"
  VERSION_PUBLICATION="$INDEX_CANDIDATE
create [--tag] [$VERSION] [$IMAGE@$INDEX_DIGEST]"
  NOETIC_MOVE="create [--tag] [$IMAGE:noetic] [$IMAGE@$INDEX_DIGEST]"
  COMPLETE_PUBLICATION="$VERSION_PUBLICATION
$NOETIC_MOVE"
  FIRST_PUBLICATION_COPY="copy robot-image-amd64 $IMAGE:first-publication"
  readonly EARLIER_COMMIT COMMIT LATER_COMMIT UNRELATED_COMMIT TAG_OBJECT VERSION VERSION_PUBLICATION
  readonly INDEX_CANDIDATE NOETIC_MOVE COMPLETE_PUBLICATION FIRST_PUBLICATION_COPY
}

# Step 2. The stand-ins read what to answer from the variables that run_publisher exports.
write_stand_ins() {
  mkdir "$work/bin"
  # The login file that docker login would write. publish_image.sh has to pass it to every
  # skopeo call that talks to the registry, and the skopeo stand-in refuses a call without it.
  echo '{"auths": {}}' > "$work/config.json"

  # skopeo: "inspect --raw docker://<tag>" asks the registry for a tag, "inspect --config" reads an
  # archive, "copy" writes to the registry. An archive must be named by its exact path.
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
# The architecture of an archive named by its exact path, or nothing
architecture_of() {
  case "$1" in
    "docker-archive:$ARCHIVES/robot-image-amd64/robot-image.tar.gz") echo amd64 ;;
    "docker-archive:$ARCHIVES/robot-image-arm64/robot-image.tar.gz") echo arm64 ;;
  esac
}
if [ "$1 $2" = "inspect --raw" ]; then
  has_option --authfile "$AUTH_FILE" "$@" || unexpected "$@"
  reference=${last#docker://}
  repository=${reference%:*} tag=${reference##*:}
  answer=$VERSION_ANSWER
  if [ "$tag" = noetic ]; then
    answer=$NOETIC_ANSWER
  fi
  # A first publication creates the package. From then on the registry can answer differently.
  if [ -n "$ANSWER_ONCE_THE_PACKAGE_EXISTS" ] && grep -q -F ":first-publication" "$WRITES"; then
    answer=$ANSWER_ONCE_THE_PACKAGE_EXISTS
  fi
  # The first FAILED_READS reads of each tag fail as an unreachable registry would
  echo "$tag" >> "$READS"
  if [ "$(grep -c -x -F "$tag" "$READS")" -le "$FAILED_READS" ]; then
    answer=registry_unreachable
  fi
  case "$answer" in
    present) echo '{}' ;;
    manifest_unknown) fatal "reading manifest $tag in $repository: manifest unknown" ;;
    manifest_denied) fatal "reading manifest $tag in $repository: denied" ;;
    unauthorized) fatal "reading manifest $tag in $repository: unauthorized" ;;
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
  case "$(architecture_of "$last")" in
    amd64) platform=$AMD64_ARCHIVE_PLATFORM archive_commit=$AMD64_ARCHIVE_COMMIT
      description=$AMD64_ARCHIVE_DESCRIPTION ;;
    arm64) platform=$ARM64_ARCHIVE_PLATFORM archive_commit=$ARM64_ARCHIVE_COMMIT
      description=$ARM64_ARCHIVE_DESCRIPTION ;;
    *) unexpected "$@" ;;
  esac
  jq -n --arg os "${platform%/*}" --arg architecture "${platform#*/}" --arg commit "$archive_commit" \
    --arg description "$description" \
    '{os: $os, architecture: $architecture,
      config: {Labels: ({"org.opencontainers.image.revision": $commit,
                         "org.opencontainers.image.description": $description}
                        | with_entries(select(.value != "")))}}'
elif [ "$1" = copy ]; then
  has_option --format oci "$@" || unexpected "$@"
  has_option --dest-authfile "$AUTH_FILE" "$@" || unexpected "$@"
  architecture=$(architecture_of "${*: -2:1}")
  [ -n "$architecture" ] || unexpected "$@"
  # A copy of the arm64 image fails while the file ARM64_COPY_FAILURES counts more than 0
  if [ "$architecture" = arm64 ] && [ "$(cat "$ARM64_COPY_FAILURES" 2> /dev/null || echo 0)" -gt 0 ]; then
    echo $(($(cat "$ARM64_COPY_FAILURES") - 1)) > "$ARM64_COPY_FAILURES"
    echo 'time="2026-11-23T11:40:00Z" level=fatal msg="writing blob: connection reset by peer"' >&2
    exit 1
  fi
  echo "copy robot-image-$architecture ${last#docker://}" >> "$WRITES"
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

  # docker: "buildx imagetools inspect" reads a published tag or digest, "buildx imagetools create"
  # writes one. noetic's index has NOETIC_COMMIT. Every other index has INDEX_MEDIA_TYPE,
  # INDEX_COMMIT, INDEX_DESCRIPTION and INDEX_ARM64_PLATFORM as its second image's platform, and
  # only the amd64 image with INDEX_ONLY_AMD64=true. The digests: NOETIC_DIGEST for noetic,
  # VERSION_DIGEST for the version's tag, INDEX_DIGEST for the rest, and none at all with
  # MANIFEST_WITHOUT_DIGEST=true.
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
  index_type=$INDEX_MEDIA_TYPE index_commit=$INDEX_COMMIT index_description=$INDEX_DESCRIPTION
  second_platform=$INDEX_ARM64_PLATFORM
  case "$4" in
    *:noetic) index_type=application/vnd.oci.image.index.v1+json index_commit=$NOETIC_COMMIT
      index_description=$DESCRIPTION second_platform=linux/arm64 ;;
  esac
  jq -n --arg revision "$revision" --arg commit "$index_commit" --arg description "$index_description" \
    --arg amd64 "$AMD64_DIGEST" --arg arm64 "$ARM64_DIGEST" --arg type "$index_type" \
    --arg os "${second_platform%/*}" --arg architecture "${second_platform#*/}" \
    --arg only_amd64 "$INDEX_ONLY_AMD64" \
    '{mediaType: $type,
      annotations: ({($revision): $commit, "org.opencontainers.image.description": $description}
                    | with_entries(select(.value != ""))),
      manifests: ([
        {digest: $amd64, platform: {os: "linux", architecture: "amd64"}},
        {digest: $arm64, platform: {os: $os, architecture: $architecture}}]
        | if $only_amd64 == "true" then .[:1] else . end)}'
elif [ "$6" = '{{json .Manifest}}' ]; then
  if [ "$MANIFEST_WITHOUT_DIGEST" = true ]; then
    echo '{}'
    exit 0
  fi
  case "$4" in
    *:noetic) digest=$NOETIC_DIGEST ;;
    *:noetic-*-index) digest=$INDEX_DIGEST ;;
    *:noetic-*) digest=$VERSION_DIGEST ;;
    *) digest=$INDEX_DIGEST ;;
  esac
  jq -n --arg digest "$digest" '{digest: $digest}'
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

# Run publish_image.sh with the stand-ins, in the test's git history. The first argument is the
# registry's answer to "does this tag exist?", for the version and for noetic. More arguments are
# variables for this run only, as NAME=value: NOETIC_ANSWER=... gives noetic another answer than
# the version. Sets exit_code, writes and output.
run_publisher() {
  local registry_answer=$1
  shift
  : > "$work/writes"
  : > "$work/reads"
  exit_code=0
  (cd "$work/history" &&
    env PATH="$work/bin:$PATH" DOCKER_CONFIG="$work" AUTH_FILE="$work/config.json" \
      GIT_CONFIG_GLOBAL=/dev/null GIT_CONFIG_NOSYSTEM=1 \
      GITHUB_STEP_SUMMARY=/dev/null WRITES="$work/writes" READS="$work/reads" \
      ARCHIVES="$work/archives" VERSION_ANSWER="$registry_answer" NOETIC_ANSWER="$registry_answer" \
      ANSWER_ONCE_THE_PACKAGE_EXISTS='' FAILED_READS=0 \
      COMMIT="$COMMIT" DESCRIPTION="$DESCRIPTION" AMD64_DIGEST="$AMD64_DIGEST" \
      ARM64_DIGEST="$ARM64_DIGEST" INDEX_DIGEST="$INDEX_DIGEST" NOETIC_DIGEST="$INDEX_DIGEST" \
      VERSION_DIGEST="$INDEX_DIGEST" MANIFEST_WITHOUT_DIGEST=false \
      NOETIC_COMMIT="$COMMIT" ARM64_IMAGE_COMMIT="$COMMIT" INDEX_ARM64_PLATFORM=linux/arm64 \
      INDEX_MEDIA_TYPE=application/vnd.oci.image.index.v1+json INDEX_COMMIT="$COMMIT" \
      INDEX_DESCRIPTION="$DESCRIPTION" INDEX_ONLY_AMD64=false ARM64_COPY_FAILURES="$work/arm64-copy-failures" \
      AMD64_ARCHIVE_PLATFORM=linux/amd64 AMD64_ARCHIVE_COMMIT="$COMMIT" \
      AMD64_ARCHIVE_DESCRIPTION="$DESCRIPTION" \
      ARM64_ARCHIVE_PLATFORM=linux/arm64 ARM64_ARCHIVE_COMMIT="$COMMIT" \
      ARM64_ARCHIVE_DESCRIPTION="$DESCRIPTION" \
      FIRST_PUBLICATION=false "$@" \
      "$PUBLISHER" "$IMAGE" "$COMMIT" "$work/archives") > "$work/output" 2>&1 || exit_code=$?
  writes=$(cat "$work/writes")
  output=$(cat "$work/output")
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

assert_output_contains() {
  if ! grep -q -F -- "$1" <<<"$output"; then
    echo "  the output lacks: $1"
    return 1
  fi
}

# The registry's own "manifest unknown" is the only answer that lets the script publish

test_publisher_publishes_both_images_and_both_tags_when_the_registry_reports_manifest_unknown() {
  run_publisher manifest_unknown
  assert_exit_code_and_writes 0 "$COMPLETE_PUBLICATION"
}

test_publisher_summary_lists_the_image_of_each_architecture() {
  run_publisher manifest_unknown
  assert_exit_code_and_writes 0 "$COMPLETE_PUBLICATION" &&
    assert_output_contains "| linux/amd64 | \`$AMD64_DIGEST\` |" &&
    assert_output_contains "| linux/arm64 | \`$ARM64_DIGEST\` |"
}

# A read that fails is tried three times

test_publisher_publishes_when_the_third_read_of_each_tag_succeeds() {
  run_publisher manifest_unknown FAILED_READS=2
  assert_exit_code_and_writes 0 "$COMPLETE_PUBLICATION"
}

test_publisher_writes_nothing_when_all_three_reads_fail() {
  run_publisher manifest_unknown FAILED_READS=3
  assert_exit_code_and_writes 1 ""
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

# A registry that refuses the read, or doesn't know the repository, stops a normal publication,
# and the log names the refusal. A first publication answers with its own tag, and only if the
# registry refuses both reads.

test_publisher_writes_nothing_and_names_the_refusal_when_the_token_service_refuses() {
  run_publisher denied
  assert_exit_code_and_writes 1 "" &&
    assert_output_contains "$IMAGE:noetic: the token service refuses a token (403)"
}

test_publisher_writes_nothing_and_names_the_refusal_when_the_registry_denies_the_manifest() {
  run_publisher manifest_denied
  assert_exit_code_and_writes 1 "" &&
    assert_output_contains "$VERSION: the registry denies the read"
}

test_publisher_writes_nothing_and_names_the_refusal_when_the_registry_wants_another_login() {
  run_publisher unauthorized
  assert_exit_code_and_writes 1 "" &&
    assert_output_contains "$VERSION: the registry wants another login"
}

test_publisher_writes_nothing_and_names_the_refusal_when_the_registry_does_not_know_the_repository() {
  run_publisher name_unknown
  assert_exit_code_and_writes 1 "" &&
    assert_output_contains "$VERSION: the registry doesn't know the repository"
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

test_first_publication_publishes_both_images_and_both_tags_once_the_registry_answers() {
  run_publisher denied FIRST_PUBLICATION=true ANSWER_ONCE_THE_PACKAGE_EXISTS=manifest_unknown
  assert_exit_code_and_writes 0 "$FIRST_PUBLICATION_COPY
$COMPLETE_PUBLICATION"
}

# An archive that isn't the tested image of its architecture stops the publication before any
# write, also before a first publication

test_publisher_writes_nothing_when_the_arm64_archive_is_from_another_commit() {
  run_publisher manifest_unknown ARM64_ARCHIVE_COMMIT="$LATER_COMMIT"
  assert_exit_code_and_writes 1 ""
}

test_publisher_writes_nothing_when_the_amd64_archive_holds_another_architecture() {
  run_publisher manifest_unknown AMD64_ARCHIVE_PLATFORM=linux/arm64
  assert_exit_code_and_writes 1 ""
}

test_publisher_writes_nothing_when_the_arm64_archive_has_another_description() {
  run_publisher manifest_unknown ARM64_ARCHIVE_DESCRIPTION="Another image"
  assert_exit_code_and_writes 1 ""
}

test_publisher_writes_nothing_when_the_arm64_archive_has_no_description() {
  run_publisher manifest_unknown ARM64_ARCHIVE_DESCRIPTION=""
  assert_exit_code_and_writes 1 ""
}

test_publisher_writes_nothing_when_neither_archive_has_a_description() {
  run_publisher manifest_unknown AMD64_ARCHIVE_DESCRIPTION="" ARM64_ARCHIVE_DESCRIPTION=""
  assert_exit_code_and_writes 1 ""
}

test_first_publication_writes_nothing_when_the_arm64_archive_is_from_another_commit() {
  run_publisher denied FIRST_PUBLICATION=true ARM64_ARCHIVE_COMMIT="$LATER_COMMIT"
  assert_exit_code_and_writes 1 ""
}

# The version's tag goes only to a combined index that passed its check

test_publisher_gives_no_tag_of_its_own_to_an_index_with_two_images_of_one_architecture() {
  run_publisher manifest_unknown INDEX_ARM64_PLATFORM=linux/amd64
  assert_exit_code_and_writes 1 "$INDEX_CANDIDATE"
}

test_publisher_gives_no_tag_of_its_own_to_an_index_without_the_arm64_image() {
  run_publisher manifest_unknown INDEX_ONLY_AMD64=true
  assert_exit_code_and_writes 1 "$INDEX_CANDIDATE"
}

test_publisher_gives_no_tag_of_its_own_to_an_index_of_another_media_type() {
  run_publisher manifest_unknown INDEX_MEDIA_TYPE=application/vnd.docker.distribution.manifest.list.v2+json
  assert_exit_code_and_writes 1 "$INDEX_CANDIDATE"
}

test_publisher_gives_no_tag_of_its_own_to_an_index_of_another_commit() {
  run_publisher manifest_unknown INDEX_COMMIT="$LATER_COMMIT"
  assert_exit_code_and_writes 1 "$INDEX_CANDIDATE"
}

test_publisher_gives_no_tag_of_its_own_to_an_index_without_the_images_description() {
  run_publisher manifest_unknown INDEX_DESCRIPTION="Another image"
  assert_exit_code_and_writes 1 "$INDEX_CANDIDATE"
}

# Each tag that the script writes is read back, and a tag with another digest stops the job

test_publisher_leaves_noetic_when_the_version_tag_points_elsewhere_after_its_write() {
  run_publisher manifest_unknown VERSION_DIGEST="$OTHER_DIGEST"
  assert_exit_code_and_writes 1 "$VERSION_PUBLICATION"
}

test_publisher_fails_when_noetic_points_elsewhere_after_its_move() {
  run_publisher manifest_unknown NOETIC_ANSWER=manifest_unknown NOETIC_DIGEST="$OTHER_DIGEST"
  assert_exit_code_and_writes 1 "$COMPLETE_PUBLICATION"
}

test_publisher_writes_nothing_when_the_registry_gives_no_digest() {
  run_publisher present MANIFEST_WITHOUT_DIGEST=true
  assert_exit_code_and_writes 1 ""
}

# A copy that fails stops the job before any tag of its own. The rerun publishes everything.

test_publisher_publishes_everything_in_the_rerun_after_a_failed_arm64_copy() {
  local status=0
  echo 1 > "$work/arm64-copy-failures"
  run_publisher manifest_unknown
  assert_exit_code_and_writes 1 "copy robot-image-amd64 $VERSION-amd64" || status=1
  if [ "$status" -eq 0 ]; then
    run_publisher manifest_unknown
    assert_exit_code_and_writes 0 "$COMPLETE_PUBLICATION" || status=1
  fi
  rm -f "$work/arm64-copy-failures"
  return "$status"
}

# A version that is published already is checked and reused, never written again

test_publisher_writes_nothing_when_the_version_and_noetic_are_published() {
  run_publisher present
  assert_exit_code_and_writes 0 ""
}

test_publisher_writes_nothing_when_a_published_image_is_from_another_commit() {
  run_publisher present ARM64_IMAGE_COMMIT="$LATER_COMMIT"
  assert_exit_code_and_writes 1 ""
}

test_publisher_writes_nothing_when_noetic_has_this_commit_but_another_digest() {
  run_publisher present NOETIC_DIGEST="$OTHER_DIGEST"
  assert_exit_code_and_writes 1 ""
}

# noetic moves only forward in git history: to this commit if noetic is missing or at an earlier
# commit, and never back or sideways

test_publisher_moves_noetic_after_the_version_when_noetic_is_at_an_earlier_commit() {
  run_publisher manifest_unknown NOETIC_ANSWER=present NOETIC_COMMIT="$EARLIER_COMMIT"
  assert_exit_code_and_writes 0 "$COMPLETE_PUBLICATION"
}

test_publisher_publishes_only_the_version_when_noetic_is_at_a_later_commit() {
  run_publisher manifest_unknown NOETIC_ANSWER=present NOETIC_COMMIT="$LATER_COMMIT"
  assert_exit_code_and_writes 0 "$VERSION_PUBLICATION"
}

test_publisher_moves_only_noetic_when_the_version_is_published_and_noetic_is_at_an_earlier_commit() {
  run_publisher present NOETIC_COMMIT="$EARLIER_COMMIT"
  assert_exit_code_and_writes 0 "$NOETIC_MOVE"
}

test_publisher_moves_only_noetic_when_the_version_is_published_and_noetic_is_missing() {
  run_publisher present NOETIC_ANSWER=manifest_unknown
  assert_exit_code_and_writes 0 "$NOETIC_MOVE"
}

test_publisher_leaves_noetic_when_it_is_at_an_unrelated_commit() {
  run_publisher present NOETIC_COMMIT="$UNRELATED_COMMIT"
  assert_exit_code_and_writes 1 ""
}

test_publisher_leaves_noetic_when_it_has_no_commit() {
  run_publisher present NOETIC_COMMIT=""
  assert_exit_code_and_writes 1 ""
}

test_publisher_leaves_noetic_when_its_commit_is_a_name_instead_of_a_commit_id() {
  run_publisher present NOETIC_COMMIT=HEAD
  assert_exit_code_and_writes 1 ""
}

test_publisher_leaves_noetic_when_its_commit_is_the_id_of_a_tag() {
  run_publisher present NOETIC_COMMIT="$TAG_OBJECT"
  assert_exit_code_and_writes 1 ""
}

test_publisher_leaves_noetic_when_its_commit_is_not_in_the_history() {
  run_publisher present NOETIC_COMMIT="$UNKNOWN_COMMIT"
  assert_exit_code_and_writes 1 ""
}

readonly TESTS=(
  test_publisher_publishes_both_images_and_both_tags_when_the_registry_reports_manifest_unknown
  test_publisher_summary_lists_the_image_of_each_architecture
  test_publisher_publishes_when_the_third_read_of_each_tag_succeeds
  test_publisher_writes_nothing_when_all_three_reads_fail
  test_publisher_writes_nothing_when_the_login_helper_is_not_installed
  test_publisher_writes_nothing_when_the_token_service_answers_not_found
  test_publisher_writes_nothing_when_a_page_without_the_registry_error_code_answers_not_found
  test_publisher_writes_nothing_when_the_registry_cannot_be_reached
  test_first_publication_writes_nothing_when_the_login_file_cannot_be_read
  test_publisher_writes_nothing_and_names_the_refusal_when_the_token_service_refuses
  test_publisher_writes_nothing_and_names_the_refusal_when_the_registry_denies_the_manifest
  test_publisher_writes_nothing_and_names_the_refusal_when_the_registry_wants_another_login
  test_publisher_writes_nothing_and_names_the_refusal_when_the_registry_does_not_know_the_repository
  test_first_publication_writes_only_its_own_tag_when_the_registry_keeps_denying_the_read
  test_first_publication_writes_only_its_own_tag_when_the_repository_stays_unknown
  test_first_publication_writes_nothing_when_the_registry_denies_only_one_of_the_two_reads
  test_first_publication_publishes_both_images_and_both_tags_once_the_registry_answers
  test_publisher_writes_nothing_when_the_arm64_archive_is_from_another_commit
  test_publisher_writes_nothing_when_the_amd64_archive_holds_another_architecture
  test_publisher_writes_nothing_when_the_arm64_archive_has_another_description
  test_publisher_writes_nothing_when_the_arm64_archive_has_no_description
  test_publisher_writes_nothing_when_neither_archive_has_a_description
  test_first_publication_writes_nothing_when_the_arm64_archive_is_from_another_commit
  test_publisher_gives_no_tag_of_its_own_to_an_index_with_two_images_of_one_architecture
  test_publisher_gives_no_tag_of_its_own_to_an_index_without_the_arm64_image
  test_publisher_gives_no_tag_of_its_own_to_an_index_of_another_media_type
  test_publisher_gives_no_tag_of_its_own_to_an_index_of_another_commit
  test_publisher_gives_no_tag_of_its_own_to_an_index_without_the_images_description
  test_publisher_leaves_noetic_when_the_version_tag_points_elsewhere_after_its_write
  test_publisher_fails_when_noetic_points_elsewhere_after_its_move
  test_publisher_writes_nothing_when_the_registry_gives_no_digest
  test_publisher_publishes_everything_in_the_rerun_after_a_failed_arm64_copy
  test_publisher_writes_nothing_when_the_version_and_noetic_are_published
  test_publisher_writes_nothing_when_a_published_image_is_from_another_commit
  test_publisher_writes_nothing_when_noetic_has_this_commit_but_another_digest
  test_publisher_moves_noetic_after_the_version_when_noetic_is_at_an_earlier_commit
  test_publisher_publishes_only_the_version_when_noetic_is_at_a_later_commit
  test_publisher_moves_only_noetic_when_the_version_is_published_and_noetic_is_at_an_earlier_commit
  test_publisher_moves_only_noetic_when_the_version_is_published_and_noetic_is_missing
  test_publisher_leaves_noetic_when_it_is_at_an_unrelated_commit
  test_publisher_leaves_noetic_when_it_has_no_commit
  test_publisher_leaves_noetic_when_its_commit_is_a_name_instead_of_a_commit_id
  test_publisher_leaves_noetic_when_its_commit_is_the_id_of_a_tag
  test_publisher_leaves_noetic_when_its_commit_is_not_in_the_history
)

# Steps 3 and 4. A failed test prints the script's output, which says where the script stopped.
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
  make_history
  write_stand_ins
  run_tests
}

main "$@"
