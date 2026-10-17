#!/usr/bin/env bash
# Push this repository to a new, empty GitHub repository.
#
#   1. On github.com: New repository -> name it (e.g. bunker_aubo_sim), leave it EMPTY
#      (no README, no licence, no .gitignore), create it.
#   2. tools/publish_to_github.sh https://github.com/<user>/<repo>.git
#      (or the SSH form git@github.com:<user>/<repo>.git)
set -eo pipefail

URL="${1:-}"
if [ -z "$URL" ]; then
  echo "usage: $0 <github-repository-url>"
  exit 1
fi
cd "$(dirname "${BASH_SOURCE[0]}")/.."

if [ -n "$(git status --porcelain)" ]; then
  echo "There are uncommitted changes. Commit them first:"
  git status --short
  exit 1
fi

git branch -M main
if git remote get-url origin >/dev/null 2>&1; then
  git remote set-url origin "$URL"
else
  git remote add origin "$URL"
fi
git push -u origin main
echo
echo "Done: ${URL%.git}"
