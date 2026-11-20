# Set up my laptop and Steam Deck for AMR

Help me provision my own devices. Inspect existing setup first; preserve working
keys and configuration. Ask me to perform account login, MFA and device approvals.

1. **Pair locally.** Keep laptop and Deck on the same LAN. Enable Developer Mode
   on Deck. Follow Valve's current [Devkit pairing procedure](https://partner.steamgames.com/doc/steamhardware/loadgames):
   install SteamOS Devkit Client on my laptop, pair this Deck, and approve on
   the Deck. Verify key-based SSH. Adapt to my laptop's actual OS.
2. **Join AMR's tailnet.** Install Tailscale on the laptop. For SteamOS, inspect
   and follow [deck-tailscale](https://github.com/tailscale-dev/deck-tailscale),
   including update persistence. Use the company-approved account/tailnet on
   both devices. Verify their actual DNS names and node identities; do not
   invent a numbered Deck alias or reuse another person's identity.
3. **Verify remote access.** Reuse the paired SSH key over the Deck's Tailscale
   address; verify its host-key fingerprint against the locally paired device.
   Ordinary OpenSSH is sufficient; Tailscale SSH is a separate optional feature.
4. **Keep secrets private.** Use the AMR Bitwarden collection shared with my
   account. Keep private keys on my laptop and credentials out of chat/Git/logs.

Finish with a short receipt: device/OS, tailnet DNS identities, public-key and
host-key fingerprints, SSH success, and any approval still needed.

**Then stop.** Device access is not robot control authorization. App installation
is the next task: the v5 pendant is built and installed by `go run . pendant
install` from `steamdeck/real/v4` with the cell's manifest
([Tier 4](../steamdeck/real/v4/README.md#tier-4-steam-deck-v5-pendant-rebuild)).
Preserve the selected cell's configuration/Home references. Do not copy another
client's runtime or auto-arm.
