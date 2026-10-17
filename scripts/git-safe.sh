#!/bin/sh
# git for unattended work: this machine's git config is right for a human at a
# keyboard and fatal for a script.
#
#   commit.gpgsign = true    every commit wants a signature (a kgpg/pinentry
#                            prompt no script can answer)
#   core.editor    = code -w any `-e` waits for a VS Code window to close - for
#                            ever, when there is no window to close
#
# Both bite hardest inside `git rebase`/`cherry-pick`, which run
# `git commit -n -S ... -e` themselves. So the override has to be an
# *environment* variable, not `git -c`: a `-c` does not reach the child git
# process rebase spawns, GIT_CONFIG_* does.
#
#   scripts/git-safe.sh rebase -i --rebase-merges <base>
#   scripts/git-safe.sh commit -m "..."
#
# For an interactive todo list, give it a sequence editor as well:
#
#   GIT_SEQUENCE_EDITOR=<script> scripts/git-safe.sh rebase -i <base>
#
# Nothing here changes the user's own git behaviour: the values live only in
# this process and in the processes it starts.
export GIT_CONFIG_COUNT=2
export GIT_CONFIG_KEY_0=commit.gpgsign
export GIT_CONFIG_VALUE_0=false
export GIT_CONFIG_KEY_1=core.editor
export GIT_CONFIG_VALUE_1=true
exec git "$@"
