#!/bin/bash
set -e

# Install the WPILib VS Code extension for GitHub Codespaces.
# WPILib is not on the VS Code marketplace, so customizations.vscode.extensions can't be
# used. postCreateCommand runs before VS Code Server is ready so `code --install-extension`
# fails there. postAttachCommand runs after VS Code has attached, at which point the code
# CLI is live and can register the extension with the running server.
# The Dockerfile handles local devcontainers (VSIX pre-extracted into the image).
if [ "${CODESPACES}" = "true" ]; then
  # Always install via the CLI so VS Code properly registers the extension and
  # activates it without requiring a manual reload. The Dockerfile pre-extracts
  # the files, but VS Code needs a CLI install to update its extension registry;
  # without it the extension sits on disk but never activates, breaking IntelliSense.
  echo "Installing WPILib VS Code extension..."
  wget -q 'https://github.com/wpilibsuite/vscode-wpilib/releases/download/v2026.2.1/vscode-wpilib-2026.2.1.vsix' -O /tmp/vscode-wpilib.vsix
  code --install-extension /tmp/vscode-wpilib.vsix --force
  rm /tmp/vscode-wpilib.vsix

  # Same for the reload helper (see reload-helper/extension.js). Install it last:
  # it may reload the window as soon as it starts. A VSIX is a zip with two
  # manifest files plus the extension folder, so build one with Python.
  echo "Installing dev container reload helper..."
  python3 - "$(dirname "$0")/reload-helper" /tmp/reload-helper.vsix << 'PYEOF'
import os, sys, zipfile
src, out = sys.argv[1], sys.argv[2]
content_types = """<?xml version="1.0" encoding="utf-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension=".json" ContentType="application/json"/>
  <Default Extension=".js" ContentType="application/javascript"/>
  <Default Extension=".vsixmanifest" ContentType="text/xml"/>
</Types>
"""
manifest = """<?xml version="1.0" encoding="utf-8"?>
<PackageManifest Version="2.0.0" xmlns="http://schemas.microsoft.com/developer/vsx-schema/2011">
  <Metadata>
    <Identity Language="en-US" Id="devcontainer-reload-helper" Version="1.0.0" Publisher="robolancers"/>
    <DisplayName>Dev container reload helper</DisplayName>
    <Description xml:space="preserve">Reloads the window once after dev container setup.</Description>
  </Metadata>
  <Installation>
    <InstallationTarget Id="Microsoft.VisualStudio.Code"/>
  </Installation>
  <Dependencies/>
  <Assets>
    <Asset Type="Microsoft.VisualStudio.Code.Manifest" Path="extension/package.json" Addressable="true"/>
  </Assets>
</PackageManifest>
"""
with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
    z.writestr("[Content_Types].xml", content_types)
    z.writestr("extension.vsixmanifest", manifest)
    for name in os.listdir(src):
        z.write(os.path.join(src, name), "extension/" + name)
PYEOF
  code --install-extension /tmp/reload-helper.vsix --force
  rm /tmp/reload-helper.vsix
fi

# postAttachCommand only runs after every postCreateCommand has finished, so this is
# the first safe moment to reload. If install-frc-mentor.sh asked for a reload, hand
# it to the reload helper extension, which waits for workspace trust before reloading.
if [ -f ~/.cache/devcontainer-reload-pending ]; then
  rm ~/.cache/devcontainer-reload-pending
  touch ~/.cache/devcontainer-reload-window
fi
