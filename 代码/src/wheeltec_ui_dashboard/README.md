# wheeltec_ui_dashboard

Run the touchscreen dashboard with:

```bash
ros2 run wheeltec_ui_dashboard ui_dashboard
```

By default the dashboard opens on the robot's local touchscreen. Set
`K1_DASHBOARD_REMOTE=1` to display it through SSH X11 forwarding.

The family services screen provides the home map, indoor patrol, safety
alerts, smart-home controls, care records, and parent chat. Parent chat uses
`/voice_trigger`; recognized dialogue is synchronized to the phone through
`home/care/dialogue`. The one-tap SOS button publishes a structured alert on
`/home/security/alert`, which `robot_mqtt_bridge` forwards to
`home/security/alert` for the phone popup and parent SMS flow.
