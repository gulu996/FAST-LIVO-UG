# Release and application readiness

Documentation baseline: 2026-10-26; algorithm `d9497af2234d3fd368dfdc072918d81c5d59a585`.
The repository is a catkin workspace (`src/FAST_LIVO2` is the package).
Read-only remote inspection returned default branch `main` at
`b9dfc0ab47ac08edcd44c101d37dc49612c0417c`, while local main was `d9497af`.
This documentation preparation does not push any commits. Readers of the older
remote must not assume they have the algorithm described by the latest results.

## Ready now / still needed

| Item | State |
|---|---|
| English architecture, core dependency/build/run instructions | Added; statically checked, not a fresh installation test |
| GPLv2 text and upstream attribution | Present at root and package; see unresolved third-party issues |
| Public mathematical/software test entrypoints | Existing tracked native tests documented |
| 804 s results, DROP2 and geometry evidence | Versioned summaries with source hashes; private data/harness still restrict exact replay |
| GNSS, UWB and joint graph distinctions | Separate real-input and synthetic scopes documented |
| License clearance for all distributed components | **Not complete**: GPL versions, MVS wrapper, firmware and notices |
| New images / raw datasets | None added; no permission assumed |
| Fresh clone build / CI | Still needed on a clean authorized environment; no rebuild in this docs task |
| Application or remote publication | Not submitted; no push performed |

## Privacy and release review

New docs use public upstream author attribution and anonymized aggregate counts.
No site photographs, maps, exact geodetic origin or raw person/vehicle imagery are
added. Evidence manifests use relative archival identifiers, not private mount
paths. Existing configuration files contain deployment coordinates, anchors,
serial/network details and local paths; e.g. `gnss_adapter_mid360.yaml` stores an
origin and `mid360.yaml` stores local paths/anchors. Review these **existing**
files for consent before a release; this task deliberately does not change YAML.
The firmware tree includes old Keil user-project files and listings that also
need publication review. A pattern scan is not certification of the whole Git
history as credential-free. Check Git history and SDK/data rights before uploading
release archives. Do not include local untracked reports or the ZED worktree.

## Codex for Open Source preparation

The official [Codex for Open Source page](https://developers.openai.com/community/codex-for-oss)
invites core maintainers and asks applicants to explain ecosystem importance.
It describes Pro/Codex support, conditional security access and API-credit support;
acceptance is reviewed, not guaranteed by this documentation.

Prepare factual maintainer/write-access evidence, project activity and public
usage/dependents (do not invent adoption), the unresolved license action plan,
and a concrete Codex maintenance proposal: reproducible geometry/transaction
regressions, code review, issue triage and release checks. For API credits,
explain the workflow and requested resources. Supply public, permission-cleared
minimal test fixtures and a clean-build result where possible. Review the current
program form/terms at submission time. No application or message to OpenAI was
sent by this task.

## Documentation checks performed

- Internal Markdown links and code fences checked; SVG parsed as XML; evidence
  JSON parsed; root LICENSE byte-identical to upstream and the package copy
  (SHA256 `8177f97513213526df2cf6184d8ff986c675afb514d4e68a404010521b880643`).
- Both mapping launches and the recording example resolved with `roslaunch --files`;
  no nodes, sensors or replay were started.
- Core build whitelist checked against tracked package dependencies; documented
  native targets checked against CMake/source. No clean-build claim is made.
- Twelve documentation external links returned HTTP 200. The OpenAI program page
  returned 403 to a direct scripted request but was successfully read through the
  documentation browser; it is not reported as a broken URL or a guaranteed API.
- New text checked for common credential patterns; new figure is generated SVG
  containing only aggregate statistics. This is not a full-history secret audit.
