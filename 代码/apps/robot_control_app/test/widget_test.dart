import 'dart:convert';

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

import 'package:robot_control_app/main.dart';

void main() {
  test('connection settings accept a complete SSH command', () async {
    final settings = AppSettingsController.memory();

    expect(
      settings.normalizeRobotHost('ssh -Y zyc@10.26.89.188'),
      '10.26.89.188',
    );
    expect(
      settings.normalizeRobotHost('http://192.168.1.8:8090/map.png'),
      '192.168.1.8',
    );
  });

  test('map coordinate conversion preserves patrol targets', () {
    const state = HomeMapState(
      width: 800,
      height: 600,
      resolution: 0.05,
      originX: -12,
      originY: -8,
      originYaw: 0.25,
      version: 1,
      robotAvailable: true,
      robotX: 1,
      robotY: 2,
      robotYaw: 0.5,
      navigationState: 'running',
    );
    const target = Offset(3.25, -1.75);
    final restored = state.pixelToWorld(state.worldToPixel(target));
    expect(restored.dx, closeTo(target.dx, 1e-9));
    expect(restored.dy, closeTo(target.dy, 1e-9));
    final stale = state.withoutRobot();
    expect(stale.robotAvailable, isFalse);
    expect(stale.robotX, state.robotX);
    expect(stale.robotY, state.robotY);
  });

  testWidgets('smart home page exposes family actions without robot stop', (
    tester,
  ) async {
    final commands = <String>[];

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.dark(),
        home: SmartHomePage(onCommand: (command, _) => commands.add(command)),
      ),
    );

    expect(find.text('家居联动'), findsOneWidget);
    expect(find.text('打开灯光'), findsOneWidget);
    expect(find.text('开启通风'), findsOneWidget);
    expect(find.text('立即停止'), findsNothing);

    await tester.ensureVisible(find.text('打开灯光'));
    await tester.tap(find.text('打开灯光'));
    await tester.pumpAndSettle();
    expect(commands, contains('LIGHT_ON'));
  });

  testWidgets('child care arm panel exposes binocular bottle grab', (
    tester,
  ) async {
    final commands = <String>[];

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.dark(),
        home: Scaffold(
          body: SafeArea(
            child: ArmControlPanel(onArmCommand: commands.add, dense: true),
          ),
        ),
      ),
    );

    expect(find.text('抓取'), findsOneWidget);
    expect(find.text('抓取调参'), findsOneWidget);
    expect(find.text('六关节姿态'), findsNothing);
    expect(find.text('用于看护场景中的轻量协助'), findsNothing);
    await tester.tap(find.text('抓取'));
    expect(commands, contains('GRAB_BOTTLE'));

    await tester.tap(find.text('抓取调参'));
    await tester.pumpAndSettle();
    expect(find.text('抓取偏移调参'), findsOneWidget);
    expect(find.text('-0.070'), findsOneWidget);
    expect(find.text('-0.030'), findsOneWidget);
    expect(find.text('0.050'), findsOneWidget);
    await tester.tap(find.text('应用'));
    await tester.pump();
    expect(tester.takeException(), isNull);
    await tester.pumpAndSettle();
    expect(tester.takeException(), isNull);
    final offsets = jsonDecode(commands.last) as Map<String, dynamic>;
    expect(offsets['type'], 'set_offsets');
    expect(offsets['x_offset'], -0.07);
    expect(offsets['y_offset'], -0.03);
    expect(offsets['z_offset'], 0.05);
  });

  testWidgets('family services and environment details support English', (
    tester,
  ) async {
    void noop() {}

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.dark(),
        home: FamilyServicesPage(
          isEnglish: true,
          onOpenMap: noop,
          onOpenChildCare: noop,
          onOpenPatrol: noop,
          onOpenSafety: noop,
          onOpenSmartHome: noop,
          onOpenRecords: noop,
        ),
      ),
    );

    expect(find.text('Family Services'), findsOneWidget);
    expect(find.text('Home Map'), findsOneWidget);
    expect(find.text('Child Care'), findsOneWidget);
    expect(find.text('家庭服务'), findsNothing);

    final current = ValueNotifier(
      EnvironmentSnapshot(
        time: DateTime(2026, 7, 14),
        temperature: 26.5,
        humidity: 54.2,
        formaldehyde: 0.03,
        pm25: 12,
        co2: 480,
        voc: 0.12,
      ),
    );
    final history = ValueNotifier<List<EnvironmentSnapshot>>([current.value]);
    final online = ValueNotifier(true);

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.dark(),
        home: EnvironmentDetailPage(
          environmentNotifier: current,
          historyNotifier: history,
          onlineNotifier: online,
          onRefresh: () {},
          onClearHistory: () async {
            history.value = [];
          },
          isEnglish: true,
        ),
      ),
    );

    expect(find.text('Home Environment'), findsOneWidget);
    expect(find.text('Indoor Air Overview'), findsOneWidget);
    expect(find.byTooltip('Clear history'), findsOneWidget);
    await tester.tap(find.byTooltip('Clear history'));
    await tester.pumpAndSettle();
    await tester.tap(find.text('Clear'));
    await tester.pumpAndSettle();
    expect(history.value, isEmpty);
    await tester.drag(find.byType(ListView).first, const Offset(0, -520));
    await tester.pumpAndSettle();
    expect(find.text('History'), findsOneWidget);
    expect(find.text('家庭环境'), findsNothing);

    await tester.pumpWidget(const SizedBox.shrink());
    current.dispose();
    history.dispose();
    online.dispose();
  });

  testWidgets('all family service detail pages use English', (tester) async {
    tester.view.devicePixelRatio = 3;
    tester.view.physicalSize = const Size(1260, 2800);
    addTearDown(() {
      tester.view.resetDevicePixelRatio();
      tester.view.resetPhysicalSize();
    });
    final messages = ValueNotifier<List<Map<String, String>>>([]);
    final patrolStatus = ValueNotifier<Map<String, dynamic>>({
      'state': 'idle',
      'message': 'Ready',
      'room': '',
    });
    final navigationStatus = ValueNotifier<Map<String, dynamic>>({
      'state': 'stopped',
      'message': 'Stopped',
    });
    final armStatus = ValueNotifier<Map<String, dynamic>>({
      'event': 'idle',
      'message': '',
    });
    final settings = AppSettingsController.memory();
    void command(String _) {}

    final pages = <(Widget, String)>[
      (
        SlamMapPage(
          isEnglish: true,
          isConnected: false,
          connectionText: 'Offline',
          mapBaseUrl: 'http://127.0.0.1:1',
          settings: settings,
          onNavigate: command,
          onSaveRoom: (_, __) {},
          onStartNavigation: command,
          navigationStatusNotifier: navigationStatus,
          patrolStatusNotifier: patrolStatus,
        ),
        'Home Map',
      ),
      (
        CameraStreamPage(
          isEnglish: true,
          onSpeedCommand: (_, __) {},
          onLateralCommand: (_) {},
          onArmCommand: command,
          onParentAudio: (_, __) {},
          dialogueNotifier: messages,
          armStatusNotifier: armStatus,
          cameraBaseUrl: 'http://127.0.0.1:1',
        ),
        'Child Care',
      ),
      (
        IndoorPatrolPage(
          isEnglish: true,
          isConnected: false,
          settings: settings,
          mapBaseUrl: 'http://127.0.0.1:1',
          statusNotifier: patrolStatus,
          onCommand: command,
        ),
        'Indoor Patrol',
      ),
      (
        const SafetyCenterPage(isEnglish: true, latestAlert: null, alerts: []),
        'Safety Alerts',
      ),
      (SmartHomePage(isEnglish: true, onCommand: (_, __) {}), 'Smart Home'),
      (
        CareRecordsPage(isEnglish: true, dialogueNotifier: messages),
        'Care Records',
      ),
    ];

    for (final page in pages) {
      await tester.pumpWidget(
        MaterialApp(theme: ThemeData.dark(), home: page.$1),
      );
      await tester.pump();
      expect(find.text(page.$2), findsOneWidget);
      expect(tester.takeException(), isNull, reason: page.$2);
    }

    await tester.pumpWidget(const SizedBox.shrink());
    messages.dispose();
    patrolStatus.dispose();
    navigationStatus.dispose();
    armStatus.dispose();
  });

  testWidgets('indoor patrol renders robot status instead of a local timer', (
    tester,
  ) async {
    tester.view.devicePixelRatio = 3;
    tester.view.physicalSize = const Size(1260, 2800);
    addTearDown(() {
      tester.view.resetDevicePixelRatio();
      tester.view.resetPhysicalSize();
    });
    final commands = <String>[];
    final settings = AppSettingsController.memory();
    final status = ValueNotifier<Map<String, dynamic>>({
      'state': 'idle',
      'message': '等待开始路径巡查',
      'room': '',
      'checked_rooms': <String>[],
      'failed_rooms': <String>[],
    });

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.dark(),
        home: IndoorPatrolPage(
          isConnected: true,
          settings: settings,
          mapBaseUrl: 'http://127.0.0.1:1',
          initialPath: const [
            PatrolWaypoint(name: '巡查点 1', x: 1.2, y: -0.4, yaw: 0.5),
          ],
          statusNotifier: status,
          onCommand: commands.add,
        ),
      ),
    );

    expect(find.text('默认房间巡查'), findsOneWidget);
    expect(find.text('厨房'), findsOneWidget);
    expect(find.text('主卧'), findsOneWidget);
    await tester.tap(find.text('开始默认房间巡查'));
    expect(commands, ['START']);
    commands.clear();

    await tester.tap(find.text('路径巡查'));
    final start = jsonDecode(commands.single) as Map<String, dynamic>;
    expect(start['command'], 'START_PATH');
    expect((start['waypoints'] as List).length, 1);

    status.value = {
      'state': 'navigating',
      'message': '正在前往巡查点 1，剩余 1.2 米',
      'room': '巡查点 1',
      'checked_rooms': <String>[],
      'failed_rooms': <String>[],
    };
    await tester.pump();

    expect(find.text('正在前往巡查点 1，剩余 1.2 米'), findsOneWidget);
    expect(find.byIcon(Icons.radar_rounded), findsOneWidget);

    await tester.tap(find.text('停止巡查'));
    expect(commands.last, 'STOP');

    await tester.pumpWidget(const SizedBox.shrink());
    status.dispose();
  });

  testWidgets('home map reports live and current patrol arrival status', (
    tester,
  ) async {
    tester.view.devicePixelRatio = 3;
    tester.view.physicalSize = const Size(1260, 2800);
    addTearDown(() {
      tester.view.resetDevicePixelRatio();
      tester.view.resetPhysicalSize();
    });
    final patrolStatus = ValueNotifier<Map<String, dynamic>>({
      'state': 'idle',
      'message': '等待目标',
      'room': '',
    });
    final navigationStatus = ValueNotifier<Map<String, dynamic>>({
      'state': 'running',
      'message': '导航已就绪',
    });

    Widget buildMap() => MaterialApp(
      theme: ThemeData.dark(),
      home: SlamMapPage(
        isConnected: true,
        connectionText: '已连接',
        mapBaseUrl: 'http://127.0.0.1:1',
        settings: AppSettingsController.memory(),
        onNavigate: (_) {},
        onSaveRoom: (_, __) {},
        onStartNavigation: (_) {},
        navigationStatusNotifier: navigationStatus,
        patrolStatusNotifier: patrolStatus,
      ),
    );

    await tester.pumpWidget(buildMap());
    await tester.pump();
    expect(find.text('已到达客厅'), findsNothing);

    patrolStatus.value = {
      'state': 'inspecting',
      'message': '已到达客厅，开始巡查',
      'room': '客厅',
      'timestamp': '2026-07-16T22:45:00+0800',
    };
    await tester.pump();

    expect(find.text('已到达客厅'), findsWidgets);
    expect(find.byIcon(Icons.check_circle_rounded), findsWidgets);

    await tester.pumpWidget(const SizedBox.shrink());
    await tester.pump();
    await tester.pumpWidget(buildMap());
    await tester.pump();
    expect(find.text('已到达客厅'), findsWidgets);

    await tester.pumpWidget(const SizedBox.shrink());
    patrolStatus.dispose();
    navigationStatus.dispose();
  });

  testWidgets('lateral control repeats until release and reports bottle', (
    tester,
  ) async {
    tester.view.devicePixelRatio = 3;
    tester.view.physicalSize = const Size(1260, 2800);
    addTearDown(() {
      tester.view.resetDevicePixelRatio();
      tester.view.resetPhysicalSize();
    });
    final lateralCommands = <double>[];
    final dialogue = ValueNotifier<List<Map<String, String>>>([]);
    final armStatus = ValueNotifier<Map<String, dynamic>>({
      'event': 'idle',
      'message': '',
    });

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.dark(),
        home: CameraStreamPage(
          isEnglish: true,
          onSpeedCommand: (_, __) {},
          onLateralCommand: lateralCommands.add,
          onArmCommand: (_) {},
          onParentAudio: (_, __) {},
          dialogueNotifier: dialogue,
          armStatusNotifier: armStatus,
          cameraBaseUrl: 'http://127.0.0.1:1',
        ),
      ),
    );
    await tester.pump();

    final gesture = await tester.startGesture(
      tester.getCenter(find.text('Left')),
    );
    await tester.pump(const Duration(milliseconds: 520));
    expect(
      lateralCommands.where((value) => value == 0.35).length,
      greaterThanOrEqualTo(3),
    );
    await gesture.up();
    await tester.pump();
    expect(lateralCommands.last, 0.0);

    armStatus.value = {
      'event': 'bottle_detected',
      'message': '已识别到瓶子，正在确认位置',
      'timestamp': '2026-07-16T23:40:00-0400',
    };
    await tester.pump();
    expect(find.text('Bottle detected'), findsOneWidget);
    expect(find.textContaining('已识别到瓶子'), findsOneWidget);

    await tester.pumpWidget(const SizedBox.shrink());
    dialogue.dispose();
    armStatus.dispose();
  });

  testWidgets('default room opens the single-room map picker', (tester) async {
    tester.view.devicePixelRatio = 3;
    tester.view.physicalSize = const Size(1260, 2800);
    addTearDown(() {
      tester.view.resetDevicePixelRatio();
      tester.view.resetPhysicalSize();
    });
    final status = ValueNotifier<Map<String, dynamic>>({
      'state': 'idle',
      'message': '等待开始家庭巡查',
      'room': '',
    });

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.dark(),
        home: IndoorPatrolPage(
          isConnected: true,
          settings: AppSettingsController.memory(),
          mapBaseUrl: 'http://127.0.0.1:1',
          statusNotifier: status,
          onCommand: (_) {},
        ),
      ),
    );

    await tester.tap(find.byKey(const ValueKey('default_room_客厅')));
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));
    expect(find.text('设置客厅位置'), findsOneWidget);
    expect(find.text('保存为客厅'), findsOneWidget);

    await tester.pumpWidget(const SizedBox.shrink());
    status.dispose();
  });
}
