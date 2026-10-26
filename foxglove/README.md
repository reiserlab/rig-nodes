# foxglove/ — generic Foxglove layouts

Layouts that apply to any rig graph (a device-status board, a recorder health panel) go here.
None exist yet: every layout written so far is arena-specific (`arena-fictrac.json`,
`arena-bias.json`, the okey-cokey dashboard) and stays in webDisplayTools' `rig/foxglove/`.
`rig_nodes.foxglove_json` rewrites a String-topic bag into a JSON MCAP that any layout can plot.
