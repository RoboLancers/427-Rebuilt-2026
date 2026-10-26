#!/bin/bash
# Installs Claude Code's CLI and the FRC Mentor plugin (copilot-port branch).
# Runs alongside post-create.sh (see postCreateCommand in devcontainer.json), so a
# failure in the WPILib setup can't skip it.
#
# The VS Code extension reads the same ~/.claude settings, so a user-scope install
# here puts the extension's Claude panel in mentor mode.
# frc-mentor is private, so git needs credentials to clone it:
# - Locally: your host's git credentials, which Dev Containers forwards.
# - Codespaces: the Codespace's own token. frc-mentor is in the same org, and
#   devcontainer.json (customizations.codespaces.repositories) requests read access to it.
# CLAUDE_CODE_PLUGIN_PREFER_HTTPS (set in devcontainer.json) skips the SSH attempt.
#
# Safe to rerun by hand: bash .devcontainer/install-frc-mentor.sh
set -e

FRC_MENTOR_MARKETPLACE="RoboLancers/frc-mentor#copilot-port"
CLAUDE="$HOME/.local/bin/claude"

if [ ! -x "$CLAUDE" ]; then
  curl -fsSL https://claude.ai/install.sh | bash
fi

if ! "$CLAUDE" plugin marketplace list 2>/dev/null | grep -q "frc-mentor"; then
  if ! "$CLAUDE" plugin marketplace add "$FRC_MENTOR_MARKETPLACE"; then
    echo "WARNING: couldn't add the frc-mentor marketplace. Check that you can read" >&2
    echo "github.com/RoboLancers/frc-mentor (in Codespaces: that you authorized access" >&2
    echo "to it when creating the Codespace), then rerun: bash .devcontainer/install-frc-mentor.sh" >&2
    exit 1
  fi
fi
"$CLAUDE" plugin install frc-mentor@frc-mentor --scope user

# Managed settings override every other settings file, so a "false" in the repo's
# .claude/settings.json (or a settings.local.json) can't turn the plugin off here.
sudo mkdir -p /etc/claude-code
echo '{ "enabledPlugins": { "frc-mentor@frc-mentor": true } }' \
  | sudo tee /etc/claude-code/managed-settings.json > /dev/null
echo "FRC Mentor plugin installed"

# Ask for one window reload, so the Claude panel starts a fresh session with the
# plugin loaded. post-attach.sh turns this into the actual reload signal once all
# setup is done, so the reload never interrupts the WPILib setup.
mkdir -p ~/.cache
touch ~/.cache/devcontainer-reload-pending
