# Debug Session: controller-download-stuck

- Status: OPEN
- Symptom: 下位机手动重置后，屏幕长时间停在 `downloaded`，上位机未能自动把控制器拉回 `program` 模式。
- Scope: `runtime` 自动恢复链路、`controller_probe`、`serial_wrap`、`pydownload`
- Notes: 本阶段仅做证据采集与插桩，不改业务逻辑。
