#!/bin/bash
# Installs Claude Code's CLI and the FRC Mentor plugin (copilot-port branch).
# Runs alongside post-create.sh (see postCreateCommand in devcontainer.json), so a
# failure in the WPILib setup can't skip it.
#
# The VS Code extension reads the same ~/.claude settings, so a user-scope install
# here puts the extension's Claude panel in mentor mode.
# frc-mentor is private, so git needs credentials to clone it:
# - Locally: your host's git credentials, which Dev Containers forwards.
# - Codespaces: the Codespace's own token can't read it, because Codespaces only grants
#   access to repos in the same organization (this repo is in RoboLancers-Java-Learning,
#   frc-mentor is in RoboLancers). Set a Codespaces secret named FRC_MENTOR_TOKEN instead:
#   a fine-grained token with read-only "Contents" access to RoboLancers/frc-mentor.
# CLAUDE_CODE_PLUGIN_PREFER_HTTPS (set in devcontainer.json) skips the SSH attempt.
#
# Safe to rerun by hand: bash .devcontainer/install-frc-mentor.sh
set -e

FRC_MENTOR_MARKETPLACE="RoboLancers/frc-mentor#copilot-port"
CLAUDE="$HOME/.local/bin/claude"

# If the token is set, use it for git in this script only. These GIT_CONFIG_* variables
# add config for child processes (Claude runs git): the empty helper clears any other
# credential helper (like the Codespace's), then the next one hands git the token.
# Nothing is written to disk, so the token isn't saved in git or Claude settings.
if [ -n "${FRC_MENTOR_TOKEN:-}" ]; then
  export GIT_CONFIG_COUNT=2
  export GIT_CONFIG_KEY_0="credential.https://github.com.helper"
  export GIT_CONFIG_VALUE_0=""
  export GIT_CONFIG_KEY_1="credential.https://github.com.helper"
  export GIT_CONFIG_VALUE_1='!f() { echo username=x-access-token; echo "password=${FRC_MENTOR_TOKEN}"; }; f'
fi

if [ ! -x "$CLAUDE" ]; then
  curl -fsSL https://claude.ai/install.sh | bash
fi

if ! "$CLAUDE" plugin marketplace list 2>/dev/null | grep -q "frc-mentor"; then
  if ! "$CLAUDE" plugin marketplace add "$FRC_MENTOR_MARKETPLACE"; then
    echo "WARNING: couldn't add the frc-mentor marketplace. Check that you can read" >&2
    echo "github.com/RoboLancers/frc-mentor (in Codespaces: that the FRC_MENTOR_TOKEN" >&2
    echo "secret is set), then rerun: bash .devcontainer/install-frc-mentor.sh" >&2
    exit 1
  fi
fi
"$CLAUDE" plugin install frc-mentor@frc-mentor --scope user
echo "FRC Mentor plugin installed"

# Ask for one window reload, so the Claude panel starts a fresh session with the
# plugin loaded. post-attach.sh turns this into the actual reload signal once all
# setup is done, so the reload never interrupts the WPILib setup.
mkdir -p ~/.cache
touch ~/.cache/devcontainer-reload-pending
