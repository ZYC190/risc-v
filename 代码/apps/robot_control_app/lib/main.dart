import 'dart:convert';
import 'dart:async';
import 'dart:io';
import 'dart:math';
import 'package:crypto/crypto.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:mqtt_client/mqtt_client.dart';
import 'package:mqtt_client/mqtt_server_client.dart';

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();
  final settings = await AppSettingsController.load();
  runApp(MyApp(settings: settings));
}

class MyApp extends StatelessWidget {
  final AppSettingsController settings;

  const MyApp({super.key, required this.settings});

  @override
  Widget build(BuildContext context) {
    return AnimatedBuilder(
      animation: settings,
      builder: (context, _) {
        final accent = settings.accentColor;
        return MaterialApp(
          key: ValueKey(settings.currentUser ?? 'signed-out'),
          debugShowCheckedModeBanner: false,
          title: settings.isEnglish ? 'Family Guardian' : '家庭守护',
          theme: ThemeData.dark().copyWith(
            scaffoldBackgroundColor: const Color(0xff0d1117),
            colorScheme: ColorScheme.dark(
              primary: accent,
              secondary: const Color(0xff22c55e),
              surface: const Color(0xff151b24),
            ),
            appBarTheme: const AppBarTheme(
              backgroundColor: Color(0xff10151d),
              elevation: 0,
            ),
            inputDecorationTheme: InputDecorationTheme(
              filled: true,
              fillColor: const Color(0xff151b24),
              border: OutlineInputBorder(
                borderRadius: BorderRadius.circular(8),
                borderSide: const BorderSide(color: Color(0xff334155)),
              ),
              enabledBorder: OutlineInputBorder(
                borderRadius: BorderRadius.circular(8),
                borderSide: const BorderSide(color: Color(0xff334155)),
              ),
              focusedBorder: OutlineInputBorder(
                borderRadius: BorderRadius.circular(8),
                borderSide: BorderSide(color: accent, width: 1.5),
              ),
            ),
          ),
          home: settings.currentUser == null
              ? AuthPage(settings: settings)
              : RobotControlDashboard(settings: settings),
        );
      },
    );
  }
}

class AppSettingsController extends ChangeNotifier {
  static const MethodChannel _channel = MethodChannel(
    'robot_control_app/native',
  );
  static const int _defaultAccent = 0xff38bdf8;

  final Map<String, dynamic> _values;

  AppSettingsController._(this._values);

  factory AppSettingsController.memory([Map<String, dynamic>? values]) =>
      AppSettingsController._(Map<String, dynamic>.from(values ?? const {}));

  static Future<AppSettingsController> load() async {
    try {
      final values = await _channel.invokeMapMethod<String, dynamic>(
        'loadPreferences',
      );
      return AppSettingsController._(Map<String, dynamic>.from(values ?? {}));
    } on MissingPluginException {
      return AppSettingsController._({});
    } catch (error) {
      debugPrint('读取本地设置失败: $error');
      return AppSettingsController._({});
    }
  }

  String get brokerHost =>
      (_values['broker_host'] as String?)?.trim().isNotEmpty == true
      ? (_values['broker_host'] as String).trim()
      : '10.26.89.188';
  int get mqttPort => int.tryParse('${_values['mqtt_port'] ?? ''}') ?? 1883;
  int get mapPort => int.tryParse('${_values['map_port'] ?? ''}') ?? 8090;
  int get cameraPort => int.tryParse('${_values['camera_port'] ?? ''}') ?? 8080;
  String get parentPhone => (_values['parent_phone'] as String?) ?? '';
  String get languageCode => (_values['language'] as String?) ?? 'zh';
  bool get isEnglish => languageCode == 'en';
  int get accentColorValue =>
      int.tryParse('${_values['accent_color'] ?? ''}') ?? _defaultAccent;
  Color get accentColor => Color(accentColorValue);
  String? get currentUser {
    final value = (_values['current_user'] as String?)?.trim() ?? '';
    return value.isEmpty ? null : value;
  }

  Map<String, String> get _accounts {
    final raw = (_values['accounts'] as String?) ?? '{}';
    try {
      final decoded = jsonDecode(raw);
      if (decoded is Map) {
        return decoded.map(
          (key, value) => MapEntry(key.toString(), value.toString()),
        );
      }
    } catch (_) {}
    return {};
  }

  bool get hasAccounts => _accounts.isNotEmpty;

  List<EnvironmentSnapshot> get environmentHistory {
    final raw = (_values['environment_history'] as String?) ?? '[]';
    try {
      final decoded = jsonDecode(raw);
      if (decoded is List) {
        return decoded
            .whereType<Map>()
            .map(
              (item) =>
                  EnvironmentSnapshot.fromMap(Map<String, dynamic>.from(item)),
            )
            .toList();
      }
    } catch (error) {
      debugPrint('读取环境历史失败: $error');
    }
    return [];
  }

  List<PatrolWaypoint> get customPatrolPath {
    final raw = (_values['custom_patrol_path'] as String?) ?? '[]';
    try {
      final decoded = jsonDecode(raw);
      if (decoded is List) {
        return decoded
            .whereType<Map>()
            .map(
              (item) => PatrolWaypoint.fromMap(Map<String, dynamic>.from(item)),
            )
            .toList();
      }
    } catch (error) {
      debugPrint('Failed to read patrol path: $error');
    }
    return [];
  }

  Map<String, PatrolWaypoint> get defaultRoomWaypoints {
    final raw = (_values['default_room_waypoints'] as String?) ?? '{}';
    try {
      final decoded = jsonDecode(raw);
      if (decoded is Map) {
        return decoded.map(
          (key, value) => MapEntry(
            key.toString(),
            PatrolWaypoint.fromMap(Map<String, dynamic>.from(value as Map)),
          ),
        );
      }
    } catch (error) {
      debugPrint('Failed to read default room waypoints: $error');
    }
    return {};
  }

  Future<void> _set(String key, Object? value, {bool rebuild = true}) async {
    if (value == null) {
      _values.remove(key);
    } else {
      _values[key] = value;
    }
    try {
      await _channel.invokeMethod<void>('setPreference', {
        'key': key,
        'value': value,
      });
    } on MissingPluginException {
      // Widget tests use in-memory settings.
    } catch (error) {
      debugPrint('保存设置失败 ($key): $error');
    }
    if (rebuild) notifyListeners();
  }

  String normalizeRobotHost(String input) {
    var value = input.trim();
    if (value.isEmpty) return '';
    final uri = Uri.tryParse(value);
    if (uri != null && uri.hasScheme && uri.host.isNotEmpty) {
      return uri.host;
    }
    final sshHost = RegExp(r'@([^\s:/]+)').firstMatch(value);
    if (sshHost != null) return sshHost.group(1)!;
    value = value.split(RegExp(r'\s+')).last;
    if (value.contains('@')) value = value.split('@').last;
    value = value.replaceFirst(RegExp(r'^https?://'), '');
    value = value.split('/').first;
    if (value.contains(':')) value = value.split(':').first;
    return value.trim();
  }

  Future<void> saveConnection({
    required String hostInput,
    required int mqttPort,
    required int mapPort,
    required int cameraPort,
  }) async {
    final host = normalizeRobotHost(hostInput);
    await _set('broker_host', host, rebuild: false);
    await _set('mqtt_port', '$mqttPort', rebuild: false);
    await _set('map_port', '$mapPort', rebuild: false);
    await _set('camera_port', '$cameraPort', rebuild: false);
    notifyListeners();
  }

  Future<void> setParentPhone(String value) =>
      _set('parent_phone', value.trim());

  Future<void> setLanguage(String value) => _set('language', value);

  Future<void> setAccentColor(Color value) =>
      _set('accent_color', '${value.toARGB32()}');

  String _passwordDigest(String username, String password) {
    final normalized = username.trim().toLowerCase();
    return sha256
        .convert(utf8.encode('family-guardian::$normalized::$password'))
        .toString();
  }

  Future<String?> register(String username, String password) async {
    final cleanUser = username.trim();
    if (cleanUser.length < 3) return '账号至少需要 3 个字符';
    if (password.length < 6) return '密码至少需要 6 位';
    final accounts = _accounts;
    final key = cleanUser.toLowerCase();
    if (accounts.containsKey(key)) return '这个账号已经注册';
    accounts[key] = _passwordDigest(cleanUser, password);
    await _set('accounts', jsonEncode(accounts), rebuild: false);
    await _set('current_user', cleanUser);
    return null;
  }

  Future<String?> login(String username, String password) async {
    final cleanUser = username.trim();
    final expected = _accounts[cleanUser.toLowerCase()];
    if (expected == null) return '账号不存在，请先注册';
    if (expected != _passwordDigest(cleanUser, password)) return '密码不正确';
    await _set('current_user', cleanUser);
    return null;
  }

  Future<void> logout() => _set('current_user', null);

  Future<void> saveEnvironmentHistory(List<EnvironmentSnapshot> values) async {
    final trimmed = values.length > 1008
        ? values.sublist(values.length - 1008)
        : values;
    await _set(
      'environment_history',
      jsonEncode(trimmed.map((item) => item.toMap()).toList()),
      rebuild: false,
    );
  }

  Future<void> saveCustomPatrolPath(List<PatrolWaypoint> values) => _set(
    'custom_patrol_path',
    jsonEncode(values.map((item) => item.toMap()).toList()),
    rebuild: false,
  );

  Future<void> saveDefaultRoomWaypoint(PatrolWaypoint waypoint) async {
    final values = defaultRoomWaypoints;
    values[waypoint.name] = waypoint;
    await _set(
      'default_room_waypoints',
      jsonEncode(values.map((key, value) => MapEntry(key, value.toMap()))),
      rebuild: false,
    );
  }
}

class PatrolWaypoint {
  final String name;
  final double x;
  final double y;
  final double yaw;

  const PatrolWaypoint({
    required this.name,
    required this.x,
    required this.y,
    required this.yaw,
  });

  factory PatrolWaypoint.fromMap(Map<String, dynamic> map) => PatrolWaypoint(
    name: (map['name'] ?? '').toString(),
    x: (map['x'] as num?)?.toDouble() ?? 0,
    y: (map['y'] as num?)?.toDouble() ?? 0,
    yaw: (map['yaw'] as num?)?.toDouble() ?? 0,
  );

  Map<String, dynamic> toMap() => {
    'name': name,
    'x': double.parse(x.toStringAsFixed(4)),
    'y': double.parse(y.toStringAsFixed(4)),
    'yaw': double.parse(yaw.toStringAsFixed(4)),
  };
}

class EnvironmentSnapshot {
  final DateTime time;
  final double temperature;
  final double humidity;
  final double formaldehyde;
  final double pm25;
  final double co2;
  final double voc;

  const EnvironmentSnapshot({
    required this.time,
    required this.temperature,
    required this.humidity,
    required this.formaldehyde,
    required this.pm25,
    required this.co2,
    required this.voc,
  });

  factory EnvironmentSnapshot.fromMap(Map<String, dynamic> map) {
    double number(String key, double fallback) =>
        double.tryParse('${map[key] ?? ''}') ?? fallback;
    return EnvironmentSnapshot(
      time:
          DateTime.tryParse('${map['time'] ?? ''}') ??
          DateTime.fromMillisecondsSinceEpoch(0),
      temperature: number('temperature', 0),
      humidity: number('humidity', 0),
      formaldehyde: number('formaldehyde', 0),
      pm25: number('pm25', 0),
      co2: number('co2', 0),
      voc: number('voc', 0),
    );
  }

  bool get isLegacyPlaceholder =>
      (temperature - 26.5).abs() < 0.001 &&
      (humidity - 54.2).abs() < 0.001 &&
      (formaldehyde - 0.03).abs() < 0.001 &&
      (pm25 - 18).abs() < 0.001 &&
      (co2 - 520).abs() < 0.001 &&
      (voc - 0.18).abs() < 0.001;

  Map<String, dynamic> toMap() => {
    'time': time.toIso8601String(),
    'temperature': temperature,
    'humidity': humidity,
    'formaldehyde': formaldehyde,
    'pm25': pm25,
    'co2': co2,
    'voc': voc,
  };
}

class AuthPage extends StatefulWidget {
  final AppSettingsController settings;

  const AuthPage({super.key, required this.settings});

  @override
  State<AuthPage> createState() => _AuthPageState();
}

class _AuthPageState extends State<AuthPage> {
  final _usernameController = TextEditingController();
  final _passwordController = TextEditingController();
  final _confirmController = TextEditingController();
  late bool _registerMode;
  bool _obscurePassword = true;
  bool _busy = false;
  String? _error;

  @override
  void initState() {
    super.initState();
    _registerMode = !widget.settings.hasAccounts;
  }

  @override
  void dispose() {
    _usernameController.dispose();
    _passwordController.dispose();
    _confirmController.dispose();
    super.dispose();
  }

  Future<void> _submit() async {
    FocusScope.of(context).unfocus();
    final password = _passwordController.text;
    if (_registerMode && password != _confirmController.text) {
      setState(() => _error = '两次输入的密码不一致');
      return;
    }
    setState(() {
      _busy = true;
      _error = null;
    });
    final error = _registerMode
        ? await widget.settings.register(_usernameController.text, password)
        : await widget.settings.login(_usernameController.text, password);
    if (!mounted) return;
    setState(() {
      _busy = false;
      _error = error;
    });
  }

  @override
  Widget build(BuildContext context) {
    final english = widget.settings.isEnglish;
    final accent = Theme.of(context).colorScheme.primary;
    return Scaffold(
      body: SafeArea(
        child: SingleChildScrollView(
          child: Column(
            children: [
              SizedBox(
                height: 235,
                width: double.infinity,
                child: Stack(
                  fit: StackFit.expand,
                  children: [
                    Image.asset(
                      'assets/images/indoor_patrol.png',
                      fit: BoxFit.cover,
                    ),
                    Align(
                      alignment: Alignment.bottomCenter,
                      child: Container(
                        width: double.infinity,
                        padding: const EdgeInsets.fromLTRB(22, 26, 22, 18),
                        color: const Color(0xdd0b1220),
                        child: Column(
                          mainAxisSize: MainAxisSize.min,
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: [
                            Text(
                              english ? 'Family Guardian' : '家庭守护',
                              style: const TextStyle(
                                fontSize: 27,
                                fontWeight: FontWeight.w900,
                              ),
                            ),
                            const SizedBox(height: 4),
                            Text(
                              english
                                  ? 'A safer, calmer home within reach'
                                  : '让家庭安全、巡查和儿童看护触手可及',
                              style: const TextStyle(color: Colors.white70),
                            ),
                          ],
                        ),
                      ),
                    ),
                    Positioned(
                      top: 8,
                      right: 8,
                      child: IconButton.filledTonal(
                        tooltip: english ? '中文' : 'English',
                        onPressed: () =>
                            widget.settings.setLanguage(english ? 'zh' : 'en'),
                        icon: const Icon(Icons.language_rounded),
                      ),
                    ),
                  ],
                ),
              ),
              Padding(
                padding: const EdgeInsets.fromLTRB(22, 24, 22, 30),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.stretch,
                  children: [
                    SegmentedButton<bool>(
                      segments: [
                        ButtonSegment(
                          value: false,
                          icon: const Icon(Icons.login_rounded),
                          label: Text(english ? 'Sign in' : '登录'),
                        ),
                        ButtonSegment(
                          value: true,
                          icon: const Icon(Icons.person_add_alt_1_rounded),
                          label: Text(english ? 'Register' : '注册'),
                        ),
                      ],
                      selected: {_registerMode},
                      onSelectionChanged: (value) => setState(() {
                        _registerMode = value.first;
                        _error = null;
                      }),
                    ),
                    const SizedBox(height: 22),
                    TextField(
                      controller: _usernameController,
                      textInputAction: TextInputAction.next,
                      decoration: InputDecoration(
                        labelText: english ? 'Account' : '账号',
                        hintText: english ? 'Enter your account' : '请输入家庭账号',
                        prefixIcon: const Icon(Icons.person_outline_rounded),
                      ),
                    ),
                    const SizedBox(height: 14),
                    TextField(
                      controller: _passwordController,
                      obscureText: _obscurePassword,
                      textInputAction: _registerMode
                          ? TextInputAction.next
                          : TextInputAction.done,
                      onSubmitted: (_) {
                        if (!_registerMode) _submit();
                      },
                      decoration: InputDecoration(
                        labelText: english ? 'Password' : '密码',
                        hintText: english
                            ? 'At least 6 characters'
                            : '至少输入 6 位密码',
                        prefixIcon: const Icon(Icons.lock_outline_rounded),
                        suffixIcon: IconButton(
                          tooltip: _obscurePassword ? '显示密码' : '隐藏密码',
                          onPressed: () => setState(
                            () => _obscurePassword = !_obscurePassword,
                          ),
                          icon: Icon(
                            _obscurePassword
                                ? Icons.visibility_rounded
                                : Icons.visibility_off_rounded,
                          ),
                        ),
                      ),
                    ),
                    if (_registerMode) ...[
                      const SizedBox(height: 14),
                      TextField(
                        controller: _confirmController,
                        obscureText: _obscurePassword,
                        textInputAction: TextInputAction.done,
                        onSubmitted: (_) => _submit(),
                        decoration: InputDecoration(
                          labelText: english ? 'Confirm password' : '确认密码',
                          hintText: english ? 'Enter it again' : '请再次输入密码',
                          prefixIcon: const Icon(Icons.verified_user_outlined),
                        ),
                      ),
                    ],
                    if (_error != null) ...[
                      const SizedBox(height: 12),
                      Text(
                        _error!,
                        style: const TextStyle(color: Colors.redAccent),
                      ),
                    ],
                    const SizedBox(height: 22),
                    FilledButton.icon(
                      onPressed: _busy ? null : _submit,
                      icon: _busy
                          ? const SizedBox(
                              width: 18,
                              height: 18,
                              child: CircularProgressIndicator(strokeWidth: 2),
                            )
                          : Icon(
                              _registerMode
                                  ? Icons.person_add_alt_1_rounded
                                  : Icons.login_rounded,
                            ),
                      label: Text(
                        _registerMode
                            ? (english ? 'Create account' : '创建家庭账号')
                            : (english ? 'Sign in' : '登录家庭守护'),
                      ),
                      style: FilledButton.styleFrom(
                        backgroundColor: accent,
                        foregroundColor: Colors.black,
                        minimumSize: const Size.fromHeight(52),
                        shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(8),
                        ),
                      ),
                    ),
                    const SizedBox(height: 12),
                    Text(
                      english
                          ? 'Account data is stored only on this phone.'
                          : '账号资料仅保存在本机，不会上传到云端。',
                      textAlign: TextAlign.center,
                      style: const TextStyle(
                        color: Colors.white38,
                        fontSize: 12,
                      ),
                    ),
                  ],
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}

class RobotControlDashboard extends StatefulWidget {
  final AppSettingsController settings;

  const RobotControlDashboard({super.key, required this.settings});

  @override
  State<RobotControlDashboard> createState() => _RobotControlDashboardState();
}

class _RobotControlDashboardState extends State<RobotControlDashboard> {
  static const MethodChannel _nativeChannel = MethodChannel(
    'robot_control_app/native',
  );

  String get broker => widget.settings.brokerHost;
  int get port => widget.settings.mqttPort;
  final String speedTopic = 'phone/cmd_vel'; // 速度指令 MQTT 话题
  final String parentTalkTopic = 'home/care/parent_talk'; // 家长远程发话，只播报不聊天
  final String alertTopic = 'home/security/alert'; // 家庭安全预警
  final String environmentTopic = 'home/environment/state'; // 温湿度实时监测
  final String environmentRequestTopic = 'home/environment/request'; // 主动刷新环境数据
  final String careDialogueTopic = 'home/care/dialogue'; // 儿童看护现场对话
  final String patrolStatusTopic = 'home/patrol/status'; // 室内巡查真实进度
  final String navigationSystemCommandTopic =
      'home/navigation/system_cmd'; // 启停导航系统
  final String navigationSystemStatusTopic =
      'home/navigation/system_status'; // 导航系统状态
  final String armStatusTopic = 'home/arm/status'; // 瓶子识别与机械臂状态
  final String armTopic = 'phone/arm_cmd'; // 机械臂控制指令
  final String esp32CmdTopic = 'edge/light/cmd'; // ESP32 灯光/蜂鸣器命令

  MqttServerClient? client;
  StreamSubscription<List<MqttReceivedMessage<MqttMessage>>>? _mqttSubscription;
  Timer? _reconnectTimer;
  Timer? _historyTimer;
  Timer? _sensorStateTimer;
  DateTime? _lastNavigationStatusAt;
  bool isConnected = false;
  bool _isConnecting = false;
  String connectionText = '正在连接';
  String smsPhoneNumber = '';
  double temp = 26.5;
  double humidity = 54.2;
  double formaldehyde = 0.03;
  double pm25 = 18;
  double co2 = 520;
  double voc = 0.18;
  DateTime? _lastEnvironmentUpdate;
  Map<String, dynamic>? lastAlert;
  final List<Map<String, dynamic>> _alertHistory = [];
  bool _alertDialogShowing = false;
  late String _connectionSignature;
  late List<EnvironmentSnapshot> _environmentHistory;
  late final ValueNotifier<EnvironmentSnapshot> _environmentNotifier;
  late final ValueNotifier<List<EnvironmentSnapshot>>
  _environmentHistoryNotifier;
  late final ValueNotifier<bool> _environmentOnlineNotifier;
  final ValueNotifier<List<Map<String, String>>> _careDialogueNotifier =
      ValueNotifier<List<Map<String, String>>>([]);
  final ValueNotifier<Map<String, dynamic>> _patrolStatusNotifier =
      ValueNotifier<Map<String, dynamic>>({
        'state': 'idle',
        'message': '等待开始家庭巡查',
        'room': '',
        'index': -1,
        'total': 0,
      });
  final ValueNotifier<Map<String, dynamic>> _navigationStatusNotifier =
      ValueNotifier<Map<String, dynamic>>({
        'state': 'stopped',
        'message': '导航系统尚未启动',
      });
  final ValueNotifier<Map<String, dynamic>> _armStatusNotifier =
      ValueNotifier<Map<String, dynamic>>({
        'event': 'idle',
        'message': '',
        'timestamp': '',
      });

  @override
  void initState() {
    super.initState();
    smsPhoneNumber = widget.settings.parentPhone;
    final savedHistory = widget.settings.environmentHistory;
    _environmentHistory = savedHistory
        .where((sample) => !sample.isLegacyPlaceholder)
        .toList();
    if (_environmentHistory.length != savedHistory.length) {
      unawaited(widget.settings.saveEnvironmentHistory(_environmentHistory));
    }
    if (_environmentHistory.isNotEmpty) {
      final latest = _environmentHistory.last;
      temp = latest.temperature;
      humidity = latest.humidity;
      formaldehyde = latest.formaldehyde;
      pm25 = latest.pm25;
      co2 = latest.co2;
      voc = latest.voc;
    }
    _environmentNotifier = ValueNotifier(_currentEnvironmentSnapshot());
    _environmentOnlineNotifier = ValueNotifier(false);
    _environmentHistoryNotifier = ValueNotifier(
      List<EnvironmentSnapshot>.from(_environmentHistory),
    );
    _connectionSignature = _currentConnectionSignature();
    widget.settings.addListener(_handleSettingsChanged);
    _connectMqtt();
    _reconnectTimer = Timer.periodic(const Duration(seconds: 5), (_) {
      if (!isConnected && !_isConnecting) {
        _connectMqtt();
      }
    });
    _historyTimer = Timer.periodic(const Duration(minutes: 10), (_) {
      _recordEnvironmentSnapshot();
    });
    _sensorStateTimer = Timer.periodic(const Duration(seconds: 2), (_) {
      final updated = _lastEnvironmentUpdate;
      final online =
          updated != null &&
          DateTime.now().difference(updated) < const Duration(seconds: 8);
      if (_environmentOnlineNotifier.value != online) {
        _environmentOnlineNotifier.value = online;
      }
      final navigationState = (_navigationStatusNotifier.value['state'] ?? '')
          .toString();
      final lastStatus = _lastNavigationStatusAt;
      if ({'running', 'starting', 'localizing'}.contains(navigationState) &&
          (lastStatus == null ||
              DateTime.now().difference(lastStatus) >
                  const Duration(seconds: 7))) {
        _navigationStatusNotifier.value = {
          'state': 'stale',
          'message': '导航后台状态已失效，请重新启动机器人主程序',
        };
      }
    });
  }

  String _currentConnectionSignature() =>
      '${widget.settings.brokerHost}:${widget.settings.mqttPort}';

  void _handleSettingsChanged() {
    final nextSignature = _currentConnectionSignature();
    final shouldReconnect = nextSignature != _connectionSignature;
    _connectionSignature = nextSignature;
    if (!mounted) return;
    setState(() {
      smsPhoneNumber = widget.settings.parentPhone;
      if (shouldReconnect) {
        isConnected = false;
        connectionText = '正在连接';
      }
    });
    if (shouldReconnect) {
      _mqttSubscription?.cancel();
      _mqttSubscription = null;
      client?.disconnect();
      client = null;
      _isConnecting = false;
      unawaited(_connectMqtt());
    }
  }

  EnvironmentSnapshot _currentEnvironmentSnapshot() => EnvironmentSnapshot(
    time: _lastEnvironmentUpdate ?? DateTime.now(),
    temperature: temp,
    humidity: humidity,
    formaldehyde: formaldehyde,
    pm25: pm25,
    co2: co2,
    voc: voc,
  );

  void _recordEnvironmentSnapshot() {
    final updated = _lastEnvironmentUpdate;
    if (updated == null ||
        DateTime.now().difference(updated) >= const Duration(seconds: 8)) {
      _environmentOnlineNotifier.value = false;
      return;
    }
    final snapshot = _currentEnvironmentSnapshot();
    _environmentNotifier.value = snapshot;
    _environmentOnlineNotifier.value = true;
    if (_environmentHistory.isNotEmpty &&
        snapshot.time.difference(_environmentHistory.last.time).abs() <
            const Duration(seconds: 30)) {
      return;
    }
    _environmentHistory.add(snapshot);
    if (_environmentHistory.length > 1008) {
      _environmentHistory.removeRange(0, _environmentHistory.length - 1008);
    }
    _environmentHistoryNotifier.value = List<EnvironmentSnapshot>.from(
      _environmentHistory,
    );
    unawaited(widget.settings.saveEnvironmentHistory(_environmentHistory));
  }

  // 核心 MQTT 连接引擎
  Future<void> _connectMqtt() async {
    if (_isConnecting) return;
    _isConnecting = true;
    if (mounted) {
      setState(() {
        connectionText = '连接中';
      });
    }

    final clientId = 'flutter_phone_${Random().nextInt(999999)}';
    client = MqttServerClient.withPort(broker, clientId, port);
    client!.logging(on: false);
    client!.keepAlivePeriod = 20;
    client!.autoReconnect = true;
    client!.resubscribeOnAutoReconnect = true;
    client!.onConnected = _onMqttConnected;
    client!.onDisconnected = _onMqttDisconnected;
    client!.onAutoReconnect = () {
      debugPrint('MQTT 自动重连中...');
      if (mounted) {
        setState(() {
          connectionText = '重连中';
        });
      }
    };
    client!.onAutoReconnected = () {
      debugPrint('MQTT 自动重连成功');
      _onMqttConnected();
    };

    final connMessage = MqttConnectMessage()
        .withClientIdentifier(clientId)
        .startClean()
        .withWillQos(MqttQos.atMostOnce);
    client!.connectionMessage = connMessage;

    try {
      await client!.connect();
      if (client!.connectionStatus!.state == MqttConnectionState.connected) {
        _onMqttConnected();
      }
    } catch (e) {
      debugPrint('MQTT 连接失败: $e');
      client?.disconnect();
      if (mounted) {
        setState(() {
          isConnected = false;
          connectionText = '未连接';
        });
      }
    } finally {
      _isConnecting = false;
    }
  }

  void _onMqttConnected() {
    if (!mounted) return;
    setState(() {
      isConnected = true;
      connectionText = '已连接';
    });
    _subscribeMqttTopics();
  }

  void _onMqttDisconnected() {
    debugPrint('MQTT 已断开');
    if (!mounted) return;
    setState(() {
      isConnected = false;
      connectionText = '已断开';
    });
    _navigationStatusNotifier.value = {
      'state': 'stale',
      'message': '机器人后台连接已断开，请检查主程序',
    };
  }

  void _subscribeMqttTopics() {
    if (client == null) return;
    client!.subscribe(alertTopic, MqttQos.atMostOnce);
    client!.subscribe(environmentTopic, MqttQos.atMostOnce);
    client!.subscribe(careDialogueTopic, MqttQos.atMostOnce);
    client!.subscribe(patrolStatusTopic, MqttQos.atMostOnce);
    client!.subscribe(navigationSystemStatusTopic, MqttQos.atMostOnce);
    client!.subscribe(armStatusTopic, MqttQos.atMostOnce);
    _mqttSubscription?.cancel();
    _mqttSubscription = client!.updates?.listen((messages) {
      for (final message in messages) {
        final publishMessage = message.payload as MqttPublishMessage;
        final payload = MqttPublishPayload.bytesToStringAsString(
          publishMessage.payload.message,
        );
        if (message.topic == alertTopic) {
          _handleSecurityAlert(payload);
        } else if (message.topic == environmentTopic) {
          _handleEnvironmentPayload(payload);
        } else if (message.topic == careDialogueTopic) {
          _handleCareDialoguePayload(payload);
        } else if (message.topic == patrolStatusTopic) {
          _handlePatrolStatusPayload(payload);
        } else if (message.topic == navigationSystemStatusTopic) {
          _handleNavigationSystemStatus(payload);
        } else if (message.topic == armStatusTopic) {
          _handleArmStatus(payload);
        }
      }
    });
    debugPrint(
      '✅ 已订阅主题: $alertTopic / $environmentTopic / '
      '$careDialogueTopic / $patrolStatusTopic',
    );
  }

  void _handleNavigationSystemStatus(String payload) {
    _lastNavigationStatusAt = DateTime.now();
    try {
      final decoded = jsonDecode(payload);
      _navigationStatusNotifier.value = decoded is Map<String, dynamic>
          ? Map<String, dynamic>.from(decoded)
          : {'state': 'unknown', 'message': payload};
    } catch (_) {
      _navigationStatusNotifier.value = {
        'state': 'unknown',
        'message': payload,
      };
    }
  }

  void _handleArmStatus(String payload) {
    try {
      final decoded = jsonDecode(payload);
      _armStatusNotifier.value = decoded is Map<String, dynamic>
          ? Map<String, dynamic>.from(decoded)
          : {'event': 'arm_status', 'message': payload, 'timestamp': ''};
    } catch (_) {
      _armStatusNotifier.value = {
        'event': 'arm_status',
        'message': payload,
        'timestamp': '',
      };
    }
  }

  void _handlePatrolStatusPayload(String payload) {
    try {
      final decoded = jsonDecode(payload);
      if (decoded is Map<String, dynamic>) {
        _patrolStatusNotifier.value = Map<String, dynamic>.from(decoded);
      }
    } catch (_) {
      _patrolStatusNotifier.value = {
        'state': 'error',
        'message': payload,
        'room': '',
        'index': -1,
        'total': 0,
      };
    }
  }

  void _handleCareDialoguePayload(String payload) {
    Map<String, dynamic> data;
    try {
      final decoded = jsonDecode(payload);
      data = decoded is Map<String, dynamic> ? decoded : {'text': payload};
    } catch (_) {
      data = {'text': payload};
    }

    final role = (data['role'] ?? data['speaker'] ?? 'user').toString();
    final text = (data['text'] ?? data['message'] ?? '').toString().trim();
    if (text.isEmpty) return;

    final item = <String, String>{
      'role': role,
      'text': text,
      'time': _formatClock(DateTime.now()),
    };
    final next = List<Map<String, String>>.from(_careDialogueNotifier.value)
      ..add(item);
    if (next.length > 30) {
      next.removeRange(0, next.length - 30);
    }
    _careDialogueNotifier.value = next;
  }

  void _handleEnvironmentPayload(String payload) {
    Map<String, dynamic> data;
    try {
      final decoded = jsonDecode(payload);
      data = decoded is Map<String, dynamic> ? decoded : {};
    } catch (_) {
      return;
    }
    _updateEnvironmentFromMap(data);
  }

  void _handleSecurityAlert(String payload) {
    Map<String, dynamic> alert;
    try {
      final decoded = jsonDecode(payload);
      alert = decoded is Map<String, dynamic>
          ? decoded
          : {'message': decoded.toString()};
    } catch (_) {
      alert = {'message': payload};
    }

    final alertType = _stringValue(alert, ['type', 'event'], '').toUpperCase();
    if (alertType == 'AIR_SECURITY_ALERT' && _isLegacyCo2OnlyAlert(alert)) {
      var environmentChanged = false;
      if (mounted) {
        setState(() {
          environmentChanged = _applyEnvironmentMap(alert);
        });
      }
      if (environmentChanged) _recordEnvironmentSnapshot();
      debugPrint('已忽略旧版仅 CO2 空气报警，CO2 仍正常显示和记录');
      return;
    }
    if (alertType == 'AIR_SECURITY_CLEAR') {
      if (!mounted) return;
      var environmentChanged = false;
      setState(() {
        lastAlert = null;
        environmentChanged = _applyEnvironmentMap(alert);
      });
      _alertHistory.add({
        ...alert,
        'received_at': DateTime.now().toIso8601String(),
      });
      if (_alertHistory.length > 50) _alertHistory.removeAt(0);
      if (environmentChanged) _recordEnvironmentSnapshot();
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(
          content: Text('家庭空气质量已恢复正常'),
          backgroundColor: Color(0xff15803d),
        ),
      );
      return;
    }

    if (!mounted) return;
    var environmentChanged = false;
    setState(() {
      lastAlert = alert;
      environmentChanged = _applyEnvironmentMap(alert);
    });
    _alertHistory.add({
      ...alert,
      'received_at': DateTime.now().toIso8601String(),
    });
    if (_alertHistory.length > 50) _alertHistory.removeAt(0);
    if (environmentChanged) _recordEnvironmentSnapshot();
    _sendSmsAlert(alert, automatic: true);

    if (_alertDialogShowing) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text(_alertText(alert)),
          backgroundColor: Colors.redAccent.withValues(alpha: 0.95),
        ),
      );
      return;
    }

    final emergencyType = _stringValue(alert, [
      'type',
      'event',
    ], '').toLowerCase();
    final emergencyLevel = _stringValue(alert, [
      'level',
      '等级',
    ], '').toLowerCase();
    final isEmergencySos =
        emergencyType == 'emergency_sos' ||
        emergencyType == 'sos' ||
        emergencyLevel == 'emergency';

    _alertDialogShowing = true;
    showDialog<void>(
      context: context,
      barrierDismissible: false,
      builder: (ctx) {
        return AlertDialog(
          backgroundColor: const Color(0xff1f1115),
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(18),
            side: const BorderSide(color: Colors.redAccent, width: 1.5),
          ),
          title: Row(
            children: [
              Container(
                padding: const EdgeInsets.all(8),
                decoration: BoxDecoration(
                  color: Colors.redAccent.withValues(alpha: 0.18),
                  shape: BoxShape.circle,
                ),
                child: const Icon(
                  Icons.warning_amber_rounded,
                  color: Colors.redAccent,
                ),
              ),
              const SizedBox(width: 10),
              Expanded(
                child: Text(
                  isEmergencySos ? '机器人紧急求救' : '家庭安全预警',
                  style: const TextStyle(
                    color: Colors.redAccent,
                    fontWeight: FontWeight.bold,
                  ),
                ),
              ),
            ],
          ),
          content: Text(
            _alertDetailText(alert),
            style: const TextStyle(color: Colors.white70, height: 1.5),
          ),
          actions: [
            TextButton.icon(
              onPressed: () => _sendSmsAlert(alert),
              icon: const Icon(Icons.sms_rounded),
              label: const Text('发送短信'),
              style: TextButton.styleFrom(foregroundColor: Colors.redAccent),
            ),
            TextButton.icon(
              onPressed: () {
                _mqttPublish(esp32CmdTopic, 'ALARM_OFF');
                _mqttPublish(esp32CmdTopic, 'FAN_OFF');
                Navigator.pop(ctx);
              },
              icon: const Icon(Icons.volume_off_rounded),
              label: const Text('关闭报警/风扇'),
              style: TextButton.styleFrom(foregroundColor: Colors.amber),
            ),
            TextButton(
              onPressed: () => Navigator.pop(ctx),
              child: const Text(
                '知道了',
                style: TextStyle(color: Color(0xff00f0ff)),
              ),
            ),
          ],
        );
      },
    ).whenComplete(() {
      _alertDialogShowing = false;
    });
  }

  bool _isLegacyCo2OnlyAlert(Map<String, dynamic> alert) {
    final reason = _stringValue(alert, [
      'message',
      'reason',
      'detail',
      '异常项',
    ], '').toLowerCase();
    if (!reason.contains('co2') && !reason.contains('二氧化碳')) return false;
    return ![
      '甲醛',
      'hcho',
      'voc',
      'pm2.5',
      'pm10',
      '烟雾',
      '燃气',
      '一氧化碳',
      'sos',
      '求救',
    ].any(reason.contains);
  }

  double? _parseDouble(dynamic value) {
    if (value is num) return value.toDouble();
    if (value is String) return double.tryParse(value);
    return null;
  }

  void _updateEnvironmentFromMap(Map<String, dynamic> data) {
    if (!mounted) return;
    var changed = false;
    setState(() {
      changed = _applyEnvironmentMap(data);
    });
    if (changed) _recordEnvironmentSnapshot();
  }

  bool _applyEnvironmentMap(Map<String, dynamic> data) {
    final nested = data['data'];
    final source = nested is Map<String, dynamic> ? nested : data;
    final parsedTemp = _parseDouble(
      source['temperature'] ?? source['temp'] ?? source['温度'],
    );
    final parsedHumidity = _parseDouble(
      source['humidity'] ?? source['humi'] ?? source['湿度'],
    );
    final parsedFormaldehyde = _parseDouble(
      source['formaldehyde'] ?? source['hcho'] ?? source['甲醛'],
    );
    final parsedPm25 = _parseDouble(
      source['pm25'] ?? source['pm2_5'] ?? source['PM2.5'],
    );
    final parsedCo2 = _parseDouble(
      source['co2'] ?? source['CO2'] ?? source['二氧化碳'],
    );
    final parsedVoc = _parseDouble(
      source['voc'] ?? source['tvoc'] ?? source['VOC'],
    );
    if (parsedTemp != null) temp = parsedTemp;
    if (parsedHumidity != null) humidity = parsedHumidity;
    if (parsedFormaldehyde != null) formaldehyde = parsedFormaldehyde;
    if (parsedPm25 != null) pm25 = parsedPm25;
    if (parsedCo2 != null) co2 = parsedCo2;
    if (parsedVoc != null) voc = parsedVoc;
    final changed =
        parsedTemp != null ||
        parsedHumidity != null ||
        parsedFormaldehyde != null ||
        parsedPm25 != null ||
        parsedCo2 != null ||
        parsedVoc != null;
    if (changed) {
      _lastEnvironmentUpdate = DateTime.now();
    }
    return changed;
  }

  String _stringValue(
    Map<String, dynamic> data,
    List<String> keys, [
    String fallback = '--',
  ]) {
    for (final key in keys) {
      final value = data[key];
      if (value != null && value.toString().trim().isNotEmpty) {
        return value.toString();
      }
    }
    return fallback;
  }

  String _alertText(Map<String, dynamic> alert) {
    return _stringValue(alert, ['message', '消息', 'title', '标题'], '检测到家庭环境异常');
  }

  String _alertDetailText(Map<String, dynamic> alert) {
    final level = _stringValue(alert, ['level', '等级'], 'warning');
    final message = _alertText(alert);
    final location = _stringValue(alert, ['location', '位置'], '家庭环境');
    final suggestion = _stringValue(alert, [
      'suggestion',
      '建议',
    ], '请立即检查现场并保持通风');
    final abnormal = _stringValue(alert, ['abnormal', '异常项'], '空气指标异常');
    final time = _stringValue(alert, [
      'timestamp',
      'time',
      '时间',
    ], DateTime.now().toIso8601String());
    return '等级：$level\n位置：$location\n异常：$abnormal\n提示：$message\n建议：$suggestion\n时间：$time';
  }

  Future<void> _configureSmsNumber() async {
    final controller = TextEditingController(text: smsPhoneNumber);
    final phone = await showDialog<String>(
      context: context,
      builder: (ctx) => AlertDialog(
        backgroundColor: const Color(0xff161b22),
        title: const Text(
          '设置短信接收号码',
          style: TextStyle(color: Color(0xff00f0ff)),
        ),
        content: TextField(
          controller: controller,
          autofocus: true,
          keyboardType: TextInputType.phone,
          style: const TextStyle(color: Colors.white),
          decoration: const InputDecoration(
            hintText: '输入你的手机号',
            hintStyle: TextStyle(color: Colors.white30),
            enabledBorder: UnderlineInputBorder(
              borderSide: BorderSide(color: Color(0xff00f0ff)),
            ),
            focusedBorder: UnderlineInputBorder(
              borderSide: BorderSide(color: Color(0xff00ff66)),
            ),
          ),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(ctx),
            child: const Text('取消', style: TextStyle(color: Colors.white54)),
          ),
          TextButton(
            onPressed: () => Navigator.pop(ctx, controller.text.trim()),
            child: const Text('保存', style: TextStyle(color: Color(0xff00ff66))),
          ),
        ],
      ),
    );
    if (phone != null && phone.isNotEmpty) {
      await widget.settings.setParentPhone(phone);
      if (!mounted) return;
      setState(() {
        smsPhoneNumber = phone;
      });
      ScaffoldMessenger.of(
        context,
      ).showSnackBar(SnackBar(content: Text('短信号码已设置: $phone')));
    }
  }

  Future<void> _sendSmsAlert(
    Map<String, dynamic> alert, {
    bool automatic = false,
  }) async {
    if (smsPhoneNumber.trim().isEmpty) {
      if (!automatic && mounted) {
        await _configureSmsNumber();
        if (smsPhoneNumber.trim().isEmpty) return;
      } else {
        if (mounted) {
          ScaffoldMessenger.of(
            context,
          ).showSnackBar(const SnackBar(content: Text('预警已收到，但未设置家长短信号码')));
        }
        return;
      }
    }

    try {
      await _nativeChannel.invokeMethod('sendSms', {
        'phone': smsPhoneNumber.trim(),
        'body':
            '${_isEmergencyAlert(alert) ? '家庭机器人紧急求救' : '家庭机器人安全预警'}：${_alertText(alert)}\n${_alertDetailText(alert)}\n请及时查看现场情况。',
      });
      if (!mounted) return;
      ScaffoldMessenger.of(
        context,
      ).showSnackBar(SnackBar(content: Text('短信已提交给系统短信服务：$smsPhoneNumber')));
    } catch (e) {
      if (!mounted) return;
      ScaffoldMessenger.of(
        context,
      ).showSnackBar(SnackBar(content: Text('短信发送失败: $e')));
    }
  }

  bool _isEmergencyAlert(Map<String, dynamic> alert) {
    final type = _stringValue(alert, ['type', 'event'], '').toLowerCase();
    final level = _stringValue(alert, ['level', '等级'], '').toLowerCase();
    return type == 'emergency_sos' || type == 'sos' || level == 'emergency';
  }

  // 🔧 MQTT 发布工具：逐个字节写入，确保 UTF-8 正确编码
  void _mqttPublish(String topic, String payload) {
    if (!isConnected || client == null) return;
    final builder = MqttClientPayloadBuilder();
    final utf8Bytes = utf8.encode(payload);
    for (final byte in utf8Bytes) {
      builder.addByte(byte);
    }
    client!.publishMessage(topic, MqttQos.atMostOnce, builder.payload!);
  }

  // 🕹️ 发射速度控制指令
  void _publishSpeed(double linearX, double angularZ, [double linearY = 0.0]) {
    if (!isConnected || client == null) return;
    Map<String, dynamic> twistJson = {
      'linear_x': double.parse(linearX.toStringAsFixed(2)),
      'linear_y': double.parse(linearY.toStringAsFixed(2)),
      'angular_z': double.parse(angularZ.toStringAsFixed(2)),
    };
    _mqttPublish(speedTopic, jsonEncode(twistJson));
  }

  void _sendParentAudio(Uint8List audio, int durationMs) {
    if (!isConnected || client == null || audio.isEmpty) return;
    final msg = {
      'type': 'parent_audio',
      'role': 'parent',
      'audio_base64': base64Encode(audio),
      'mime': 'audio/mp4',
      'duration_ms': durationMs,
      'timestamp': DateTime.now().toIso8601String(),
    };
    _mqttPublish(parentTalkTopic, jsonEncode(msg));
    debugPrint('家长原声已发送：${(durationMs / 1000).toStringAsFixed(1)} 秒');
  }

  void _openCameraStream() {
    Navigator.of(context).push(
      MaterialPageRoute(
        builder: (_) => CameraStreamPage(
          isEnglish: widget.settings.isEnglish,
          onSpeedCommand: _publishSpeed,
          onLateralCommand: (linearY) => _publishSpeed(0, 0, linearY),
          onArmCommand: (payload) => _mqttPublish(armTopic, payload),
          onParentAudio: _sendParentAudio,
          dialogueNotifier: _careDialogueNotifier,
          armStatusNotifier: _armStatusNotifier,
          cameraBaseUrl:
              'http://${widget.settings.brokerHost}:${widget.settings.cameraPort}',
        ),
      ),
    );
  }

  void _openFamilyMap() {
    Navigator.of(context).push(
      MaterialPageRoute(
        builder: (_) => SlamMapPage(
          isEnglish: widget.settings.isEnglish,
          isConnected: isConnected,
          connectionText: connectionText,
          settings: widget.settings,
          mapBaseUrl:
              'http://${widget.settings.brokerHost}:${widget.settings.mapPort}',
          onNavigate: (room) =>
              _mqttPublish('home/navigation/goal', jsonEncode({'room': room})),
          onSaveRoom: (room, waypoint) => _mqttPublish(
            'home/patrol/cmd',
            jsonEncode({
              'command': 'SET_ROOM',
              'room': room,
              'waypoint': waypoint.toMap(),
            }),
          ),
          onStartNavigation: (_) => _mqttPublish(
            navigationSystemCommandTopic,
            jsonEncode({'command': 'START'}),
          ),
          navigationStatusNotifier: _navigationStatusNotifier,
          patrolStatusNotifier: _patrolStatusNotifier,
        ),
      ),
    );
  }

  void _openEnvironmentDetails() {
    Navigator.of(context).push(
      MaterialPageRoute(
        builder: (_) => EnvironmentDetailPage(
          environmentNotifier: _environmentNotifier,
          historyNotifier: _environmentHistoryNotifier,
          onlineNotifier: _environmentOnlineNotifier,
          onRefresh: _requestEnvironmentRefresh,
          onClearHistory: _clearEnvironmentHistory,
          isEnglish: widget.settings.isEnglish,
        ),
      ),
    );
  }

  void _requestEnvironmentRefresh() {
    if (!isConnected) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text(
            widget.settings.isEnglish
                ? 'Robot is offline; refresh was not sent'
                : '机器人未连接，无法刷新环境数据',
          ),
        ),
      );
      return;
    }
    _mqttPublish(
      environmentRequestTopic,
      jsonEncode({
        'command': 'REFRESH',
        'timestamp': DateTime.now().toIso8601String(),
      }),
    );
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(
          widget.settings.isEnglish
              ? 'Requesting latest sensor data'
              : '正在请求最新传感器数据',
        ),
        duration: const Duration(seconds: 1),
      ),
    );
  }

  Future<void> _clearEnvironmentHistory() async {
    _environmentHistory.clear();
    _environmentHistoryNotifier.value = const <EnvironmentSnapshot>[];
    await widget.settings.saveEnvironmentHistory(const <EnvironmentSnapshot>[]);
  }

  void _openFamilyServices() {
    Navigator.of(context).push(
      MaterialPageRoute(
        builder: (_) => FamilyServicesPage(
          isEnglish: widget.settings.isEnglish,
          onOpenMap: _openFamilyMap,
          onOpenChildCare: _openCameraStream,
          onOpenPatrol: _openPatrol,
          onOpenSafety: _openSafetyCenter,
          onOpenSmartHome: _openSmartHome,
          onOpenRecords: _openCareRecords,
        ),
      ),
    );
  }

  void _openPatrol() {
    Navigator.of(context).push(
      MaterialPageRoute(
        builder: (_) => IndoorPatrolPage(
          isEnglish: widget.settings.isEnglish,
          isConnected: isConnected,
          settings: widget.settings,
          mapBaseUrl:
              'http://${widget.settings.brokerHost}:${widget.settings.mapPort}',
          statusNotifier: _patrolStatusNotifier,
          onCommand: (command) => _mqttPublish('home/patrol/cmd', command),
        ),
      ),
    );
  }

  void _openSafetyCenter() {
    Navigator.of(context).push(
      MaterialPageRoute(
        builder: (_) => SafetyCenterPage(
          isEnglish: widget.settings.isEnglish,
          latestAlert: lastAlert,
          alerts: List<Map<String, dynamic>>.from(_alertHistory),
        ),
      ),
    );
  }

  void _openSmartHome() {
    Navigator.of(context).push(
      MaterialPageRoute(
        builder: (_) => SmartHomePage(
          isEnglish: widget.settings.isEnglish,
          onCommand: _sendHomeCommand,
        ),
      ),
    );
  }

  void _openCareRecords() {
    Navigator.of(context).push(
      MaterialPageRoute(
        builder: (_) => CareRecordsPage(
          isEnglish: widget.settings.isEnglish,
          dialogueNotifier: _careDialogueNotifier,
        ),
      ),
    );
  }

  void _openSettings() {
    Navigator.of(context).push(
      MaterialPageRoute(
        builder: (_) => SettingsPage(
          settings: widget.settings,
          isConnected: isConnected,
          connectionText: connectionText,
          onReconnect: _reconnectNow,
        ),
      ),
    );
  }

  void _reconnectNow() {
    _mqttSubscription?.cancel();
    _mqttSubscription = null;
    client?.disconnect();
    client = null;
    _isConnecting = false;
    if (mounted) {
      setState(() {
        isConnected = false;
        connectionText = '正在连接';
      });
    }
    unawaited(_connectMqtt());
  }

  void _sendHomeCommand(String command, String feedback) {
    _mqttPublish(esp32CmdTopic, command);
    if (!mounted) return;
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(feedback),
        duration: const Duration(milliseconds: 900),
        behavior: SnackBarBehavior.floating,
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    final english = widget.settings.isEnglish;
    return Scaffold(
      appBar: AppBar(
        title: Text(
          english ? 'Family Guardian' : '家庭守护',
          style: const TextStyle(fontWeight: FontWeight.w800),
        ),
        centerTitle: false,
        actions: [
          Padding(
            padding: const EdgeInsets.only(right: 2),
            child: Row(
              children: [
                Container(
                  width: 8,
                  height: 8,
                  decoration: BoxDecoration(
                    color: isConnected
                        ? const Color(0xff22c55e)
                        : Colors.redAccent,
                    shape: BoxShape.circle,
                  ),
                ),
                const SizedBox(width: 6),
                Text(
                  english
                      ? (isConnected ? 'Online' : 'Offline')
                      : connectionText,
                  style: TextStyle(
                    color: isConnected ? Colors.white70 : Colors.redAccent,
                    fontSize: 11,
                  ),
                ),
              ],
            ),
          ),
          IconButton(
            tooltip: english ? 'Settings' : '设置',
            onPressed: _openSettings,
            icon: const Icon(Icons.settings_rounded),
          ),
        ],
      ),
      body: SingleChildScrollView(
        child: Padding(
          padding: const EdgeInsets.fromLTRB(16, 12, 16, 24),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              _buildRobotStatusHeader(),
              const SizedBox(height: 12),
              _buildSectionTitle(english ? 'Home status' : '家庭状态'),
              const SizedBox(height: 7),
              _buildEnvironmentPanel(),
              const SizedBox(height: 7),
              _buildSectionTitle(english ? 'Family services' : '家庭服务'),
              const SizedBox(height: 7),
              _buildFamilyServiceEntry(),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildRobotStatusHeader() {
    final english = widget.settings.isEnglish;
    final statusColor = isConnected
        ? const Color(0xff22c55e)
        : const Color(0xffef4444);
    return Container(
      width: double.infinity,
      height: 190,
      clipBehavior: Clip.antiAlias,
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: const Color(0xff263241)),
      ),
      child: Stack(
        fit: StackFit.expand,
        children: [
          Image.asset(
            'assets/images/indoor_patrol.png',
            fit: BoxFit.cover,
            alignment: Alignment.center,
          ),
          Positioned(
            top: 12,
            right: 12,
            child: _buildStatusPill(
              english
                  ? (isConnected ? 'Guardian online' : 'Waiting to connect')
                  : (isConnected ? '守护在线' : '等待连接'),
              statusColor,
            ),
          ),
          Align(
            alignment: Alignment.bottomCenter,
            child: Container(
              width: double.infinity,
              padding: const EdgeInsets.fromLTRB(16, 13, 16, 14),
              color: const Color(0xdd0b1220),
              child: Column(
                mainAxisSize: MainAxisSize.min,
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    english ? 'Family Guardian Robot' : '家庭守护机器人',
                    style: const TextStyle(
                      color: Colors.white,
                      fontSize: 21,
                      fontWeight: FontWeight.w900,
                    ),
                  ),
                  const SizedBox(height: 3),
                  Text(
                    english
                        ? 'Safety alerts · Indoor patrol · Child care'
                        : '家庭安全预警 · 室内巡查 · 儿童看护',
                    style: const TextStyle(
                      color: Color(0xffd5e6f5),
                      fontSize: 12,
                      fontWeight: FontWeight.w600,
                    ),
                  ),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildEnvironmentPanel() {
    final english = widget.settings.isEnglish;
    return ValueListenableBuilder<bool>(
      valueListenable: _environmentOnlineNotifier,
      builder: (context, sensorOnline, _) => Material(
        color: const Color(0xff111827),
        borderRadius: BorderRadius.circular(8),
        child: InkWell(
          onTap: _openEnvironmentDetails,
          borderRadius: BorderRadius.circular(8),
          child: Container(
            width: double.infinity,
            padding: const EdgeInsets.fromLTRB(11, 9, 11, 9),
            decoration: BoxDecoration(
              borderRadius: BorderRadius.circular(8),
              border: Border.all(color: const Color(0xff253044)),
            ),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Row(
                  children: [
                    Icon(
                      Icons.sensors_rounded,
                      color: Theme.of(context).colorScheme.primary,
                      size: 20,
                    ),
                    const SizedBox(width: 8),
                    Expanded(
                      child: Text(
                        english ? 'Live environment' : '环境实时监测',
                        style: const TextStyle(
                          fontSize: 14,
                          fontWeight: FontWeight.w700,
                          color: Colors.white,
                        ),
                      ),
                    ),
                    Text(
                      !sensorOnline || _lastEnvironmentUpdate == null
                          ? (english ? 'Sensor offline' : '传感器离线')
                          : '${english ? 'Updated' : '更新'} ${_formatClock(_lastEnvironmentUpdate!)}',
                      style: TextStyle(
                        color: Colors.white.withValues(alpha: 0.52),
                        fontSize: 11,
                      ),
                    ),
                  ],
                ),
                const SizedBox(height: 7),
                Row(
                  children: [
                    Expanded(
                      child: _buildSensorCard(
                        Icons.thermostat_rounded,
                        english ? 'Temperature' : '温度',
                        sensorOnline ? '${temp.toStringAsFixed(1)}°C' : '--',
                        sensorOnline
                            ? _temperatureStateText(english)
                            : (english ? 'Offline' : '离线'),
                        const Color(0xff38bdf8),
                      ),
                    ),
                    const SizedBox(width: 10),
                    Expanded(
                      child: _buildSensorCard(
                        Icons.water_drop_rounded,
                        english ? 'Humidity' : '湿度',
                        sensorOnline ? '${humidity.toStringAsFixed(1)}%' : '--',
                        sensorOnline
                            ? _humidityStateText(english)
                            : (english ? 'Offline' : '离线'),
                        const Color(0xff22c55e),
                      ),
                    ),
                  ],
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }

  Widget _buildFamilyServiceEntry() {
    final english = widget.settings.isEnglish;
    return Material(
      color: const Color(0xff151b24),
      borderRadius: BorderRadius.circular(8),
      child: InkWell(
        onTap: _openFamilyServices,
        borderRadius: BorderRadius.circular(8),
        child: Container(
          height: 330,
          clipBehavior: Clip.antiAlias,
          decoration: BoxDecoration(
            borderRadius: BorderRadius.circular(8),
            border: Border.all(
              color: Theme.of(
                context,
              ).colorScheme.primary.withValues(alpha: 0.38),
            ),
          ),
          child: Stack(
            fit: StackFit.expand,
            children: [
              Image.asset(
                'assets/images/family_services.png',
                fit: BoxFit.cover,
                alignment: Alignment.center,
              ),
              Align(
                alignment: Alignment.bottomCenter,
                child: Container(
                  width: double.infinity,
                  padding: const EdgeInsets.fromLTRB(15, 12, 12, 13),
                  color: const Color(0xe6111822),
                  child: Row(
                    children: [
                      Expanded(
                        child: Column(
                          mainAxisSize: MainAxisSize.min,
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: [
                            Text(
                              english ? 'Open family services' : '进入家庭服务',
                              style: const TextStyle(
                                color: Colors.white,
                                fontSize: 17,
                                fontWeight: FontWeight.w800,
                              ),
                            ),
                            const SizedBox(height: 3),
                            Text(
                              english
                                  ? 'Map · Child care · Patrol · Safety and more'
                                  : '家庭地图 · 儿童看护 · 室内巡查等 6 项服务',
                              style: const TextStyle(
                                color: Colors.white70,
                                fontSize: 12,
                              ),
                            ),
                          ],
                        ),
                      ),
                      Icon(
                        Icons.arrow_forward_rounded,
                        color: Theme.of(context).colorScheme.primary,
                      ),
                    ],
                  ),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildSectionTitle(String title) {
    return Row(
      children: [
        Container(
          width: 3,
          height: 16,
          decoration: BoxDecoration(
            color: const Color(0xff38bdf8),
            borderRadius: BorderRadius.circular(2),
          ),
        ),
        const SizedBox(width: 8),
        Text(
          title,
          style: const TextStyle(
            color: Colors.white,
            fontSize: 15,
            fontWeight: FontWeight.w700,
          ),
        ),
      ],
    );
  }

  Widget _buildStatusPill(String text, Color color) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
      decoration: BoxDecoration(
        color: color.withValues(alpha: 0.12),
        borderRadius: BorderRadius.circular(999),
        border: Border.all(color: color.withValues(alpha: 0.45)),
      ),
      child: Text(
        text,
        style: TextStyle(
          color: color,
          fontSize: 12,
          fontWeight: FontWeight.w700,
        ),
      ),
    );
  }

  String _temperatureStateText(bool english) {
    if (temp >= 35) return english ? 'High' : '偏高';
    if (temp <= 10) return english ? 'Low' : '偏低';
    return english ? 'Normal' : '正常';
  }

  String _humidityStateText(bool english) {
    if (humidity >= 75) return english ? 'Humid' : '偏潮湿';
    if (humidity <= 25) return english ? 'Dry' : '偏干燥';
    return english ? 'Comfort' : '舒适';
  }

  String _formatClock(DateTime value) {
    String two(int n) => n.toString().padLeft(2, '0');
    return '${two(value.hour)}:${two(value.minute)}:${two(value.second)}';
  }

  Widget _buildSensorCard(
    IconData icon,
    String title,
    String value,
    String state,
    Color themeColor,
  ) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 9, vertical: 7),
      decoration: BoxDecoration(
        color: const Color(0xff0f172a),
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: themeColor.withValues(alpha: 0.22)),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Icon(icon, color: themeColor, size: 17),
              const SizedBox(width: 6),
              Expanded(
                child: Text(
                  title,
                  style: TextStyle(
                    color: Colors.white.withValues(alpha: 0.62),
                    fontSize: 12,
                  ),
                ),
              ),
              Container(
                padding: const EdgeInsets.symmetric(horizontal: 6, vertical: 2),
                decoration: BoxDecoration(
                  color: themeColor.withValues(alpha: 0.12),
                  borderRadius: BorderRadius.circular(999),
                  border: Border.all(color: themeColor.withValues(alpha: 0.38)),
                ),
                child: Text(
                  state,
                  style: TextStyle(
                    color: themeColor,
                    fontSize: 10,
                    fontWeight: FontWeight.w700,
                  ),
                ),
              ),
            ],
          ),
          const SizedBox(height: 5),
          SizedBox(
            height: 25,
            child: FittedBox(
              alignment: Alignment.centerLeft,
              fit: BoxFit.scaleDown,
              child: Text(
                value,
                maxLines: 1,
                style: TextStyle(
                  color: themeColor,
                  fontSize: 24,
                  fontWeight: FontWeight.w800,
                  fontFamily: 'monospace',
                  height: 1,
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }

  @override
  void dispose() {
    widget.settings.removeListener(_handleSettingsChanged);
    _reconnectTimer?.cancel();
    _historyTimer?.cancel();
    _sensorStateTimer?.cancel();
    _mqttSubscription?.cancel();
    _environmentNotifier.dispose();
    _environmentHistoryNotifier.dispose();
    _environmentOnlineNotifier.dispose();
    _careDialogueNotifier.dispose();
    _patrolStatusNotifier.dispose();
    _navigationStatusNotifier.dispose();
    _armStatusNotifier.dispose();
    client?.disconnect();
    super.dispose();
  }
}

enum EnvironmentMetric { temperature, humidity, formaldehyde, pm25, co2, voc }

extension EnvironmentMetricInfo on EnvironmentMetric {
  String get label => switch (this) {
    EnvironmentMetric.temperature => '温度',
    EnvironmentMetric.humidity => '湿度',
    EnvironmentMetric.formaldehyde => '甲醛',
    EnvironmentMetric.pm25 => 'PM2.5',
    EnvironmentMetric.co2 => 'CO₂',
    EnvironmentMetric.voc => 'VOC',
  };

  String localizedLabel(bool english) {
    if (!english) return label;
    return switch (this) {
      EnvironmentMetric.temperature => 'Temperature',
      EnvironmentMetric.humidity => 'Humidity',
      EnvironmentMetric.formaldehyde => 'Formaldehyde',
      EnvironmentMetric.pm25 => 'PM2.5',
      EnvironmentMetric.co2 => 'CO₂',
      EnvironmentMetric.voc => 'VOC',
    };
  }

  String get unit => switch (this) {
    EnvironmentMetric.temperature => '°C',
    EnvironmentMetric.humidity => '%',
    EnvironmentMetric.formaldehyde => 'mg/m³',
    EnvironmentMetric.pm25 => 'μg/m³',
    EnvironmentMetric.co2 => 'ppm',
    EnvironmentMetric.voc => 'mg/m³',
  };

  Color get color => switch (this) {
    EnvironmentMetric.temperature => const Color(0xff38bdf8),
    EnvironmentMetric.humidity => const Color(0xff22c55e),
    EnvironmentMetric.formaldehyde => const Color(0xfff59e0b),
    EnvironmentMetric.pm25 => const Color(0xffa78bfa),
    EnvironmentMetric.co2 => const Color(0xff14b8a6),
    EnvironmentMetric.voc => const Color(0xfff97316),
  };

  IconData get icon => switch (this) {
    EnvironmentMetric.temperature => Icons.thermostat_rounded,
    EnvironmentMetric.humidity => Icons.water_drop_rounded,
    EnvironmentMetric.formaldehyde => Icons.science_rounded,
    EnvironmentMetric.pm25 => Icons.blur_on_rounded,
    EnvironmentMetric.co2 => Icons.cloud_outlined,
    EnvironmentMetric.voc => Icons.air_rounded,
  };

  double valueOf(EnvironmentSnapshot sample) => switch (this) {
    EnvironmentMetric.temperature => sample.temperature,
    EnvironmentMetric.humidity => sample.humidity,
    EnvironmentMetric.formaldehyde => sample.formaldehyde,
    EnvironmentMetric.pm25 => sample.pm25,
    EnvironmentMetric.co2 => sample.co2,
    EnvironmentMetric.voc => sample.voc,
  };

  int get fractionDigits => switch (this) {
    EnvironmentMetric.formaldehyde || EnvironmentMetric.voc => 2,
    EnvironmentMetric.temperature || EnvironmentMetric.humidity => 1,
    _ => 0,
  };
}

class EnvironmentDetailPage extends StatefulWidget {
  final ValueNotifier<EnvironmentSnapshot> environmentNotifier;
  final ValueNotifier<List<EnvironmentSnapshot>> historyNotifier;
  final ValueNotifier<bool> onlineNotifier;
  final VoidCallback onRefresh;
  final Future<void> Function() onClearHistory;
  final bool isEnglish;

  const EnvironmentDetailPage({
    super.key,
    required this.environmentNotifier,
    required this.historyNotifier,
    required this.onlineNotifier,
    required this.onRefresh,
    required this.onClearHistory,
    this.isEnglish = false,
  });

  @override
  State<EnvironmentDetailPage> createState() => _EnvironmentDetailPageState();
}

class _EnvironmentDetailPageState extends State<EnvironmentDetailPage> {
  EnvironmentMetric _selectedMetric = EnvironmentMetric.temperature;
  int _rangeHours = 24;

  List<EnvironmentSnapshot> _filteredHistory(List<EnvironmentSnapshot> values) {
    final cutoff = DateTime.now().subtract(Duration(hours: _rangeHours));
    final filtered = values.where((item) => item.time.isAfter(cutoff)).toList();
    return filtered.isEmpty ? values : filtered;
  }

  Future<void> _confirmClearHistory() async {
    if (widget.historyNotifier.value.isEmpty) return;
    final confirmed = await showDialog<bool>(
      context: context,
      builder: (context) => AlertDialog(
        title: Text(widget.isEnglish ? 'Clear history?' : '清除历史记录？'),
        content: Text(
          widget.isEnglish
              ? 'Saved environment samples on this phone will be deleted.'
              : '将删除这台手机上保存的全部家庭环境历史数据。',
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(false),
            child: Text(widget.isEnglish ? 'Cancel' : '取消'),
          ),
          FilledButton(
            onPressed: () => Navigator.of(context).pop(true),
            child: Text(widget.isEnglish ? 'Clear' : '清除'),
          ),
        ],
      ),
    );
    if (confirmed != true) return;
    await widget.onClearHistory();
    if (!mounted) return;
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(content: Text(widget.isEnglish ? 'History cleared' : '历史记录已清除')),
    );
  }

  @override
  Widget build(BuildContext context) {
    final english = widget.isEnglish;
    return Scaffold(
      appBar: AppBar(
        title: Text(
          english ? 'Home Environment' : '家庭环境',
          style: const TextStyle(fontWeight: FontWeight.w800),
        ),
        actions: [
          ValueListenableBuilder<List<EnvironmentSnapshot>>(
            valueListenable: widget.historyNotifier,
            builder: (context, history, _) => IconButton(
              tooltip: english ? 'Clear history' : '清除历史记录',
              onPressed: history.isEmpty ? null : _confirmClearHistory,
              icon: const Icon(Icons.delete_sweep_rounded),
            ),
          ),
          IconButton(
            tooltip: english ? 'Refresh sensor data' : '刷新传感器数据',
            onPressed: widget.onRefresh,
            icon: const Icon(Icons.refresh_rounded),
          ),
        ],
      ),
      body: ValueListenableBuilder<bool>(
        valueListenable: widget.onlineNotifier,
        builder: (context, online, _) {
          return ValueListenableBuilder<EnvironmentSnapshot>(
            valueListenable: widget.environmentNotifier,
            builder: (context, current, _) {
              return ValueListenableBuilder<List<EnvironmentSnapshot>>(
                valueListenable: widget.historyNotifier,
                builder: (context, history, _) {
                  final filtered = _filteredHistory(history);
                  return ListView(
                    padding: const EdgeInsets.fromLTRB(16, 14, 16, 28),
                    children: [
                      SizedBox(
                        height: 170,
                        child: Stack(
                          fit: StackFit.expand,
                          children: [
                            ClipRRect(
                              borderRadius: BorderRadius.circular(8),
                              child: Image.asset(
                                'assets/images/safety_alert.png',
                                fit: BoxFit.cover,
                              ),
                            ),
                            Align(
                              alignment: Alignment.bottomCenter,
                              child: Container(
                                width: double.infinity,
                                padding: const EdgeInsets.all(14),
                                decoration: const BoxDecoration(
                                  color: Color(0xdd0b1220),
                                  borderRadius: BorderRadius.vertical(
                                    bottom: Radius.circular(8),
                                  ),
                                ),
                                child: Row(
                                  children: [
                                    Expanded(
                                      child: Column(
                                        mainAxisSize: MainAxisSize.min,
                                        crossAxisAlignment:
                                            CrossAxisAlignment.start,
                                        children: [
                                          Text(
                                            english
                                                ? 'Indoor Air Overview'
                                                : '室内空气概览',
                                            style: const TextStyle(
                                              fontSize: 18,
                                              fontWeight: FontWeight.w800,
                                            ),
                                          ),
                                          const SizedBox(height: 3),
                                          Text(
                                            english
                                                ? 'Tracking changes in your home'
                                                : '持续记录家庭环境变化',
                                            style: const TextStyle(
                                              color: Colors.white70,
                                            ),
                                          ),
                                        ],
                                      ),
                                    ),
                                    Column(
                                      mainAxisSize: MainAxisSize.min,
                                      children: [
                                        Icon(
                                          online
                                              ? Icons.sensors_rounded
                                              : Icons.sensors_off_rounded,
                                          color: online
                                              ? const Color(0xff22c55e)
                                              : Colors.orangeAccent,
                                          size: 28,
                                        ),
                                        Text(
                                          online
                                              ? (english ? 'Online' : '传感器在线')
                                              : (english ? 'Offline' : '传感器离线'),
                                          style: TextStyle(
                                            color: online
                                                ? const Color(0xff22c55e)
                                                : Colors.orangeAccent,
                                            fontSize: 10,
                                            fontWeight: FontWeight.w700,
                                          ),
                                        ),
                                      ],
                                    ),
                                  ],
                                ),
                              ),
                            ),
                          ],
                        ),
                      ),
                      const SizedBox(height: 16),
                      GridView.builder(
                        shrinkWrap: true,
                        physics: const NeverScrollableScrollPhysics(),
                        gridDelegate:
                            const SliverGridDelegateWithFixedCrossAxisCount(
                              crossAxisCount: 2,
                              crossAxisSpacing: 9,
                              mainAxisSpacing: 9,
                              childAspectRatio: 2.05,
                            ),
                        itemCount: EnvironmentMetric.values.length,
                        itemBuilder: (context, index) {
                          final metric = EnvironmentMetric.values[index];
                          return _buildMetricCard(metric, current, online);
                        },
                      ),
                      const SizedBox(height: 20),
                      Row(
                        children: [
                          Expanded(
                            child: Text(
                              english ? 'History' : '历史趋势',
                              style: const TextStyle(
                                fontSize: 17,
                                fontWeight: FontWeight.w800,
                              ),
                            ),
                          ),
                          SegmentedButton<int>(
                            showSelectedIcon: false,
                            style: const ButtonStyle(
                              visualDensity: VisualDensity.compact,
                            ),
                            segments: [
                              ButtonSegment(
                                value: 24,
                                label: Text(english ? '1 day' : '1天'),
                              ),
                              ButtonSegment(
                                value: 168,
                                label: Text(english ? '7 days' : '7天'),
                              ),
                              ButtonSegment(
                                value: 720,
                                label: Text(english ? '30 days' : '30天'),
                              ),
                            ],
                            selected: {_rangeHours},
                            onSelectionChanged: (value) =>
                                setState(() => _rangeHours = value.first),
                          ),
                        ],
                      ),
                      const SizedBox(height: 10),
                      SingleChildScrollView(
                        scrollDirection: Axis.horizontal,
                        child: Row(
                          children: EnvironmentMetric.values.map((metric) {
                            return Padding(
                              padding: const EdgeInsets.only(right: 7),
                              child: ChoiceChip(
                                label: Text(metric.localizedLabel(english)),
                                selected: _selectedMetric == metric,
                                onSelected: (_) =>
                                    setState(() => _selectedMetric = metric),
                                selectedColor: metric.color.withValues(
                                  alpha: 0.28,
                                ),
                                side: BorderSide(
                                  color: metric.color.withValues(alpha: 0.5),
                                ),
                              ),
                            );
                          }).toList(),
                        ),
                      ),
                      const SizedBox(height: 8),
                      Container(
                        height: 230,
                        padding: const EdgeInsets.fromLTRB(8, 12, 8, 4),
                        decoration: BoxDecoration(
                          color: const Color(0xff111827),
                          borderRadius: BorderRadius.circular(8),
                          border: Border.all(color: const Color(0xff263241)),
                        ),
                        child: filtered.isEmpty
                            ? Center(
                                child: Text(
                                  english
                                      ? 'Waiting for samples to draw the trend'
                                      : '等待环境数据，收到采样后会自动生成曲线',
                                  textAlign: TextAlign.center,
                                  style: TextStyle(color: Colors.white38),
                                ),
                              )
                            : CustomPaint(
                                painter: EnvironmentTrendPainter(
                                  samples: filtered,
                                  metric: _selectedMetric,
                                ),
                                child: const SizedBox.expand(),
                              ),
                      ),
                      const SizedBox(height: 8),
                      Text(
                        !online
                            ? (english
                                  ? 'Sensor offline. Showing saved history only; no new samples are being added.'
                                  : '传感器离线，以下仅为本机已保存历史，当前不会新增记录')
                            : (english
                                  ? (history.length < 2
                                        ? '${history.length} sample recorded; more data will complete the trend'
                                        : '${history.length} environment samples saved on this phone')
                                  : (history.length < 2
                                        ? '已记录 ${history.length} 次，继续采样后会形成完整趋势'
                                        : '本机已保存 ${history.length} 次环境采样')),
                        style: const TextStyle(
                          color: Color(0x75ffffff),
                          fontSize: 12,
                        ),
                      ),
                      const SizedBox(height: 20),
                      Text(
                        english ? 'Recent Records' : '最近记录',
                        style: const TextStyle(
                          fontSize: 17,
                          fontWeight: FontWeight.w800,
                        ),
                      ),
                      const SizedBox(height: 7),
                      if (history.isEmpty)
                        Padding(
                          padding: const EdgeInsets.symmetric(vertical: 26),
                          child: Center(
                            child: Text(
                              english ? 'No history yet' : '暂无历史记录',
                              style: const TextStyle(color: Colors.white38),
                            ),
                          ),
                        )
                      else
                        ...history.reversed.take(20).map(_buildHistoryRow),
                    ],
                  );
                },
              );
            },
          );
        },
      ),
    );
  }

  Widget _buildMetricCard(
    EnvironmentMetric metric,
    EnvironmentSnapshot sample,
    bool online,
  ) {
    final value = online
        ? metric.valueOf(sample).toStringAsFixed(metric.fractionDigits)
        : '--';
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 10),
      decoration: BoxDecoration(
        color: const Color(0xff151b24),
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: metric.color.withValues(alpha: 0.35)),
      ),
      child: Row(
        children: [
          Icon(metric.icon, color: metric.color, size: 23),
          const SizedBox(width: 10),
          Expanded(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  metric.localizedLabel(widget.isEnglish),
                  maxLines: 1,
                  overflow: TextOverflow.ellipsis,
                  style: const TextStyle(
                    color: Colors.white70,
                    fontSize: 14,
                    fontWeight: FontWeight.w700,
                  ),
                ),
                const SizedBox(height: 2),
                FittedBox(
                  fit: BoxFit.scaleDown,
                  alignment: Alignment.centerLeft,
                  child: Text(
                    '$value ${metric.unit}',
                    style: TextStyle(
                      color: metric.color,
                      fontSize: 18,
                      fontWeight: FontWeight.w800,
                    ),
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildHistoryRow(EnvironmentSnapshot sample) {
    String two(int value) => value.toString().padLeft(2, '0');
    final time =
        '${two(sample.time.month)}-${two(sample.time.day)} ${two(sample.time.hour)}:${two(sample.time.minute)}';
    return Container(
      margin: const EdgeInsets.only(bottom: 7),
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 11),
      decoration: BoxDecoration(
        color: const Color(0xff151b24),
        borderRadius: BorderRadius.circular(8),
      ),
      child: Row(
        children: [
          SizedBox(
            width: 82,
            child: Text(
              time,
              style: const TextStyle(color: Color(0x75ffffff), fontSize: 11),
            ),
          ),
          Expanded(
            child: Text(
              _selectedMetric.localizedLabel(widget.isEnglish),
              style: const TextStyle(fontWeight: FontWeight.w700),
            ),
          ),
          Text(
            '${_selectedMetric.valueOf(sample).toStringAsFixed(_selectedMetric.fractionDigits)} ${_selectedMetric.unit}',
            style: TextStyle(color: _selectedMetric.color, fontSize: 12),
          ),
        ],
      ),
    );
  }
}

class EnvironmentTrendPainter extends CustomPainter {
  final List<EnvironmentSnapshot> samples;
  final EnvironmentMetric metric;

  EnvironmentTrendPainter({required this.samples, required this.metric});

  @override
  void paint(Canvas canvas, Size size) {
    const left = 43.0;
    const right = 10.0;
    const top = 12.0;
    const bottom = 26.0;
    final chart = Rect.fromLTRB(
      left,
      top,
      size.width - right,
      size.height - bottom,
    );
    final values = samples.map(metric.valueOf).toList();
    var minValue = values.reduce(min);
    var maxValue = values.reduce(max);
    if ((maxValue - minValue).abs() < 0.001) {
      final padding = maxValue.abs() < 0.1 ? 0.02 : maxValue.abs() * 0.08 + 1;
      minValue -= padding;
      maxValue += padding;
    } else {
      final padding = (maxValue - minValue) * 0.14;
      minValue -= padding;
      maxValue += padding;
    }

    final gridPaint = Paint()
      ..color = Colors.white.withValues(alpha: 0.09)
      ..strokeWidth = 1;
    for (var index = 0; index <= 4; index++) {
      final y = chart.top + chart.height * index / 4;
      canvas.drawLine(Offset(chart.left, y), Offset(chart.right, y), gridPaint);
      final value = maxValue - (maxValue - minValue) * index / 4;
      final label = TextPainter(
        text: TextSpan(
          text: value.toStringAsFixed(metric.fractionDigits),
          style: const TextStyle(color: Colors.white38, fontSize: 9),
        ),
        textDirection: TextDirection.ltr,
      )..layout(maxWidth: left - 6);
      label.paint(canvas, Offset(left - label.width - 6, y - label.height / 2));
    }

    final points = <Offset>[];
    for (var index = 0; index < values.length; index++) {
      final x = values.length == 1
          ? chart.center.dx
          : chart.left + chart.width * index / (values.length - 1);
      final normalized = (values[index] - minValue) / (maxValue - minValue);
      final y = chart.bottom - chart.height * normalized;
      points.add(Offset(x, y));
    }

    if (points.length > 1) {
      final fillPath = Path()
        ..moveTo(points.first.dx, chart.bottom)
        ..lineTo(points.first.dx, points.first.dy);
      for (final point in points.skip(1)) {
        fillPath.lineTo(point.dx, point.dy);
      }
      fillPath
        ..lineTo(points.last.dx, chart.bottom)
        ..close();
      canvas.drawPath(
        fillPath,
        Paint()..color = metric.color.withValues(alpha: 0.1),
      );
      final linePath = Path()..moveTo(points.first.dx, points.first.dy);
      for (final point in points.skip(1)) {
        linePath.lineTo(point.dx, point.dy);
      }
      canvas.drawPath(
        linePath,
        Paint()
          ..color = metric.color
          ..style = PaintingStyle.stroke
          ..strokeWidth = 2.5
          ..strokeCap = StrokeCap.round
          ..strokeJoin = StrokeJoin.round,
      );
    }
    for (final point in points) {
      canvas.drawCircle(point, 3.2, Paint()..color = metric.color);
    }

    if (samples.isNotEmpty) {
      final first = _clock(samples.first.time);
      final last = _clock(samples.last.time);
      _paintAxisText(canvas, first, Offset(chart.left, chart.bottom + 7));
      final lastPainter = _textPainter(last);
      lastPainter.paint(
        canvas,
        Offset(chart.right - lastPainter.width, chart.bottom + 7),
      );
    }
  }

  String _clock(DateTime value) =>
      '${value.month}/${value.day} ${value.hour.toString().padLeft(2, '0')}:${value.minute.toString().padLeft(2, '0')}';

  TextPainter _textPainter(String text) => TextPainter(
    text: TextSpan(
      text: text,
      style: const TextStyle(color: Colors.white38, fontSize: 9),
    ),
    textDirection: TextDirection.ltr,
  )..layout();

  void _paintAxisText(Canvas canvas, String text, Offset offset) {
    _textPainter(text).paint(canvas, offset);
  }

  @override
  bool shouldRepaint(covariant EnvironmentTrendPainter oldDelegate) =>
      oldDelegate.samples != samples || oldDelegate.metric != metric;
}

class SettingsPage extends StatefulWidget {
  final AppSettingsController settings;
  final bool isConnected;
  final String connectionText;
  final VoidCallback onReconnect;

  const SettingsPage({
    super.key,
    required this.settings,
    required this.isConnected,
    required this.connectionText,
    required this.onReconnect,
  });

  @override
  State<SettingsPage> createState() => _SettingsPageState();
}

class _SettingsPageState extends State<SettingsPage> {
  late final TextEditingController _hostController;
  late final TextEditingController _mqttPortController;
  late final TextEditingController _mapPortController;
  late final TextEditingController _cameraPortController;
  late final TextEditingController _phoneController;
  bool _savingConnection = false;

  @override
  void initState() {
    super.initState();
    _hostController = TextEditingController(text: widget.settings.brokerHost);
    _mqttPortController = TextEditingController(
      text: '${widget.settings.mqttPort}',
    );
    _mapPortController = TextEditingController(
      text: '${widget.settings.mapPort}',
    );
    _cameraPortController = TextEditingController(
      text: '${widget.settings.cameraPort}',
    );
    _phoneController = TextEditingController(text: widget.settings.parentPhone);
  }

  @override
  void dispose() {
    _hostController.dispose();
    _mqttPortController.dispose();
    _mapPortController.dispose();
    _cameraPortController.dispose();
    _phoneController.dispose();
    super.dispose();
  }

  int? _validPort(TextEditingController controller) {
    final value = int.tryParse(controller.text.trim());
    if (value == null || value < 1 || value > 65535) return null;
    return value;
  }

  Future<void> _saveConnection() async {
    final host = widget.settings.normalizeRobotHost(_hostController.text);
    final mqttPort = _validPort(_mqttPortController);
    final mapPort = _validPort(_mapPortController);
    final cameraPort = _validPort(_cameraPortController);
    if (host.isEmpty ||
        mqttPort == null ||
        mapPort == null ||
        cameraPort == null) {
      ScaffoldMessenger.of(
        context,
      ).showSnackBar(const SnackBar(content: Text('请检查主机地址和端口，端口范围为 1-65535')));
      return;
    }
    setState(() => _savingConnection = true);
    final mqttChanged =
        host != widget.settings.brokerHost ||
        mqttPort != widget.settings.mqttPort;
    await widget.settings.saveConnection(
      hostInput: _hostController.text,
      mqttPort: mqttPort,
      mapPort: mapPort,
      cameraPort: cameraPort,
    );
    if (!mounted) return;
    _hostController.text = host;
    if (!mqttChanged) widget.onReconnect();
    setState(() => _savingConnection = false);
    ScaffoldMessenger.of(
      context,
    ).showSnackBar(SnackBar(content: Text('已保存并连接 $host:$mqttPort')));
  }

  Future<void> _savePhone() async {
    final phone = _phoneController.text.replaceAll(RegExp(r'\s+'), '');
    if (phone.isNotEmpty && !RegExp(r'^\+?\d{6,20}$').hasMatch(phone)) {
      ScaffoldMessenger.of(
        context,
      ).showSnackBar(const SnackBar(content: Text('请输入正确的家长手机号码')));
      return;
    }
    await widget.settings.setParentPhone(phone);
    if (!mounted) return;
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(content: Text(phone.isEmpty ? '已关闭短信通知' : '家长电话已保存')),
    );
  }

  @override
  Widget build(BuildContext context) {
    final english = widget.settings.isEnglish;
    final colors = const [
      Color(0xff38bdf8),
      Color(0xff22c55e),
      Color(0xfff59e0b),
      Color(0xffef4444),
      Color(0xffa78bfa),
    ];
    return Scaffold(
      appBar: AppBar(
        title: Text(
          english ? 'Settings' : '设置',
          style: const TextStyle(fontWeight: FontWeight.w800),
        ),
      ),
      body: ListView(
        padding: const EdgeInsets.fromLTRB(16, 14, 16, 30),
        children: [
          Theme(
            data: Theme.of(context).copyWith(dividerColor: Colors.transparent),
            child: ExpansionTile(
              leading: Icon(
                Icons.router_rounded,
                color: Theme.of(context).colorScheme.primary,
              ),
              title: Text(
                english ? 'Robot connection' : '机器人连接',
                style: const TextStyle(fontWeight: FontWeight.w800),
              ),
              subtitle: Row(
                children: [
                  Container(
                    width: 8,
                    height: 8,
                    decoration: BoxDecoration(
                      color: widget.isConnected
                          ? const Color(0xff22c55e)
                          : Colors.redAccent,
                      shape: BoxShape.circle,
                    ),
                  ),
                  const SizedBox(width: 7),
                  Expanded(
                    child: Text(
                      '${widget.isConnected ? '已连接' : widget.connectionText} · ${widget.settings.brokerHost}',
                      maxLines: 1,
                      overflow: TextOverflow.ellipsis,
                    ),
                  ),
                ],
              ),
              tilePadding: const EdgeInsets.symmetric(horizontal: 14),
              childrenPadding: const EdgeInsets.fromLTRB(14, 4, 14, 14),
              collapsedBackgroundColor: const Color(0xff151b24),
              backgroundColor: const Color(0xff151b24),
              collapsedShape: RoundedRectangleBorder(
                borderRadius: BorderRadius.circular(8),
                side: const BorderSide(color: Color(0xff263241)),
              ),
              shape: RoundedRectangleBorder(
                borderRadius: BorderRadius.circular(8),
                side: const BorderSide(color: Color(0xff263241)),
              ),
              children: [
                TextField(
                  controller: _hostController,
                  keyboardType: TextInputType.url,
                  decoration: const InputDecoration(
                    labelText: '机器人地址',
                    hintText: '可粘贴 ssh -Y zyc@10.26.89.188',
                    prefixIcon: Icon(Icons.dns_rounded),
                    helperText: '支持 IP、网址或完整 SSH 命令，将自动提取主机地址',
                  ),
                ),
                const SizedBox(height: 12),
                Row(
                  children: [
                    Expanded(
                      child: _portField(
                        _mqttPortController,
                        'MQTT',
                        Icons.hub_rounded,
                      ),
                    ),
                    const SizedBox(width: 8),
                    Expanded(
                      child: _portField(
                        _mapPortController,
                        '地图',
                        Icons.map_rounded,
                      ),
                    ),
                    const SizedBox(width: 8),
                    Expanded(
                      child: _portField(
                        _cameraPortController,
                        '摄像头',
                        Icons.videocam_rounded,
                      ),
                    ),
                  ],
                ),
                const SizedBox(height: 14),
                FilledButton.icon(
                  onPressed: _savingConnection ? null : _saveConnection,
                  icon: _savingConnection
                      ? const SizedBox(
                          width: 17,
                          height: 17,
                          child: CircularProgressIndicator(strokeWidth: 2),
                        )
                      : const Icon(Icons.sync_rounded),
                  label: const Text('保存并重新连接'),
                  style: FilledButton.styleFrom(
                    minimumSize: const Size.fromHeight(48),
                    shape: RoundedRectangleBorder(
                      borderRadius: BorderRadius.circular(8),
                    ),
                  ),
                ),
              ],
            ),
          ),
          const SizedBox(height: 22),
          _sectionTitle(Icons.sms_rounded, english ? 'Family alerts' : '家庭通知'),
          const SizedBox(height: 9),
          TextField(
            controller: _phoneController,
            keyboardType: TextInputType.phone,
            decoration: InputDecoration(
              labelText: english ? 'Parent phone number' : '家长短信号码',
              hintText: english ? 'Optional' : '用于家庭安全预警短信',
              prefixIcon: const Icon(Icons.phone_android_rounded),
              suffixIcon: IconButton(
                tooltip: english ? 'Save' : '保存',
                onPressed: _savePhone,
                icon: const Icon(Icons.save_rounded),
              ),
            ),
          ),
          const SizedBox(height: 22),
          _sectionTitle(
            Icons.palette_rounded,
            english ? 'Appearance' : '外观与语言',
          ),
          const SizedBox(height: 10),
          const Text('主题颜色', style: TextStyle(color: Colors.white60)),
          const SizedBox(height: 9),
          Wrap(
            spacing: 12,
            children: colors.map((color) {
              final selected =
                  widget.settings.accentColorValue == color.toARGB32();
              return Tooltip(
                message: '选择主题颜色',
                child: InkWell(
                  onTap: () => widget.settings.setAccentColor(color),
                  customBorder: const CircleBorder(),
                  child: Container(
                    width: 42,
                    height: 42,
                    decoration: BoxDecoration(
                      color: color,
                      shape: BoxShape.circle,
                      border: Border.all(
                        color: selected ? Colors.white : Colors.transparent,
                        width: 3,
                      ),
                    ),
                    child: selected
                        ? const Icon(Icons.check_rounded, color: Colors.black)
                        : null,
                  ),
                ),
              );
            }).toList(),
          ),
          const SizedBox(height: 15),
          SegmentedButton<String>(
            segments: const [
              ButtonSegment(value: 'zh', label: Text('中文')),
              ButtonSegment(value: 'en', label: Text('English')),
            ],
            selected: {widget.settings.languageCode},
            onSelectionChanged: (value) =>
                widget.settings.setLanguage(value.first),
          ),
          const SizedBox(height: 24),
          _sectionTitle(
            Icons.account_circle_rounded,
            english ? 'Account' : '账号',
          ),
          const SizedBox(height: 9),
          ListTile(
            tileColor: const Color(0xff151b24),
            shape: RoundedRectangleBorder(
              borderRadius: BorderRadius.circular(8),
            ),
            leading: const CircleAvatar(child: Icon(Icons.person_rounded)),
            title: Text(widget.settings.currentUser ?? '家庭账号'),
            subtitle: Text(english ? 'Stored on this phone' : '账号保存在本机'),
            trailing: TextButton.icon(
              onPressed: () async {
                await widget.settings.logout();
                if (!context.mounted) return;
                Navigator.of(context).popUntil((route) => route.isFirst);
              },
              icon: const Icon(Icons.logout_rounded),
              label: Text(english ? 'Sign out' : '退出'),
            ),
          ),
        ],
      ),
    );
  }

  Widget _sectionTitle(IconData icon, String title) {
    return Row(
      children: [
        Icon(icon, color: Theme.of(context).colorScheme.primary, size: 20),
        const SizedBox(width: 8),
        Text(
          title,
          style: const TextStyle(fontSize: 16, fontWeight: FontWeight.w800),
        ),
      ],
    );
  }

  Widget _portField(
    TextEditingController controller,
    String label,
    IconData icon,
  ) {
    return TextField(
      controller: controller,
      keyboardType: TextInputType.number,
      textAlign: TextAlign.center,
      decoration: InputDecoration(
        labelText: label,
        prefixIcon: Icon(icon, size: 18),
        prefixIconConstraints: const BoxConstraints(minWidth: 34),
        contentPadding: const EdgeInsets.symmetric(horizontal: 6, vertical: 13),
      ),
    );
  }
}

class FamilyServicesPage extends StatelessWidget {
  final bool isEnglish;
  final VoidCallback onOpenMap;
  final VoidCallback onOpenChildCare;
  final VoidCallback onOpenPatrol;
  final VoidCallback onOpenSafety;
  final VoidCallback onOpenSmartHome;
  final VoidCallback onOpenRecords;

  const FamilyServicesPage({
    super.key,
    this.isEnglish = false,
    required this.onOpenMap,
    required this.onOpenChildCare,
    required this.onOpenPatrol,
    required this.onOpenSafety,
    required this.onOpenSmartHome,
    required this.onOpenRecords,
  });

  @override
  Widget build(BuildContext context) {
    final english = isEnglish;
    final services = [
      (
        'assets/images/indoor_map.png',
        english ? 'Home Map' : '家庭地图',
        english
            ? 'Choose the living room, kitchen or bedroom'
            : '选择客厅、厨房、卧室等家庭房间',
        const Color(0xff38bdf8),
        onOpenMap,
      ),
      (
        'assets/images/child_care.png',
        english ? 'Child Care' : '儿童看护',
        english ? 'Live view, mobile care and parent voice' : '实时画面、移动看护与家长发话',
        const Color(0xff22c55e),
        onOpenChildCare,
      ),
      (
        'assets/images/indoor_patrol.png',
        english ? 'Indoor Patrol' : '室内巡查',
        english ? 'Review patrol progress room by room' : '按房间查看家庭巡查进度',
        const Color(0xff14b8a6),
        onOpenPatrol,
      ),
      (
        'assets/images/safety_alert.png',
        english ? 'Safety Alerts' : '安全预警',
        english ? 'Review home alerts and their status' : '集中查看家庭异常和处理状态',
        const Color(0xffef4444),
        onOpenSafety,
      ),
      (
        'assets/images/smart_home.png',
        english ? 'Smart Home' : '家居联动',
        english ? 'Control lights, ventilation and alarms' : '控制灯光、通风与声光提醒',
        const Color(0xfff59e0b),
        onOpenSmartHome,
      ),
      (
        'assets/images/system_status.png',
        english ? 'Care Records' : '看护记录',
        english ? 'Review robot and family conversations' : '回看机器人和家庭成员的对话',
        const Color(0xffa78bfa),
        onOpenRecords,
      ),
    ];
    return Scaffold(
      appBar: AppBar(
        title: Text(
          english ? 'Family Services' : '家庭服务',
          style: const TextStyle(fontWeight: FontWeight.w800),
        ),
      ),
      body: GridView.builder(
        padding: const EdgeInsets.fromLTRB(16, 14, 16, 24),
        gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(
          crossAxisCount: 2,
          crossAxisSpacing: 10,
          mainAxisSpacing: 10,
          childAspectRatio: 0.88,
        ),
        itemCount: services.length,
        itemBuilder: (context, index) {
          final service = services[index];
          return Material(
            color: const Color(0xff151b24),
            borderRadius: BorderRadius.circular(8),
            child: InkWell(
              onTap: service.$5,
              borderRadius: BorderRadius.circular(8),
              child: Container(
                clipBehavior: Clip.antiAlias,
                decoration: BoxDecoration(
                  borderRadius: BorderRadius.circular(8),
                  border: Border.all(color: service.$4.withValues(alpha: 0.4)),
                ),
                child: Column(
                  children: [
                    Expanded(
                      child: Image.asset(
                        service.$1,
                        width: double.infinity,
                        fit: BoxFit.cover,
                      ),
                    ),
                    Padding(
                      padding: const EdgeInsets.fromLTRB(11, 10, 8, 11),
                      child: Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          Row(
                            children: [
                              Expanded(
                                child: Text(
                                  service.$2,
                                  style: const TextStyle(
                                    fontSize: 15,
                                    fontWeight: FontWeight.w800,
                                  ),
                                ),
                              ),
                              Icon(
                                Icons.chevron_right_rounded,
                                color: service.$4,
                                size: 19,
                              ),
                            ],
                          ),
                          const SizedBox(height: 4),
                          SizedBox(
                            height: 31,
                            child: Text(
                              service.$3,
                              maxLines: 2,
                              overflow: TextOverflow.ellipsis,
                              style: TextStyle(
                                color: service.$4.withValues(alpha: 0.86),
                                fontSize: 11,
                                height: 1.35,
                              ),
                            ),
                          ),
                        ],
                      ),
                    ),
                  ],
                ),
              ),
            ),
          );
        },
      ),
    );
  }
}

class SmartHomePage extends StatelessWidget {
  final void Function(String command, String feedback) onCommand;
  final bool isEnglish;

  const SmartHomePage({
    super.key,
    required this.onCommand,
    this.isEnglish = false,
  });

  @override
  Widget build(BuildContext context) {
    final english = isEnglish;
    final actions = [
      (
        Icons.lightbulb_rounded,
        english ? 'Lights On' : '打开灯光',
        'LIGHT_ON',
        english ? 'Living room lights turned on' : '客厅灯已打开',
        const Color(0xff22c55e),
      ),
      (
        Icons.lightbulb_outline_rounded,
        english ? 'Lights Off' : '关闭灯光',
        'LIGHT_OFF',
        english ? 'Living room lights turned off' : '客厅灯已关闭',
        const Color(0xff94a3b8),
      ),
      (
        Icons.air_rounded,
        english ? 'Ventilation On' : '开启通风',
        'FAN_ON',
        english ? 'Ventilation turned on' : '室内通风已开启',
        const Color(0xff38bdf8),
      ),
      (
        Icons.air_outlined,
        english ? 'Ventilation Off' : '关闭通风',
        'FAN_OFF',
        english ? 'Ventilation turned off' : '室内通风已关闭',
        const Color(0xff94a3b8),
      ),
      (
        Icons.notifications_active_rounded,
        english ? 'Alarm On' : '声光预警',
        'ALARM_ON',
        english ? 'Sound and light alarm enabled' : '声光预警已开启',
        const Color(0xfff59e0b),
      ),
      (
        Icons.notifications_off_rounded,
        english ? 'Alarm Off' : '解除预警',
        'ALARM_OFF',
        english ? 'Sound and light alarm cleared' : '声光预警已解除',
        const Color(0xffa78bfa),
      ),
      (
        Icons.power_settings_new_rounded,
        english ? 'All Off' : '全部关闭',
        'ALL_OFF',
        english ? 'All home devices turned off' : '家庭设备已全部关闭',
        const Color(0xfff97316),
      ),
    ];
    return Scaffold(
      appBar: AppBar(
        title: Text(
          english ? 'Smart Home' : '家居联动',
          style: const TextStyle(fontWeight: FontWeight.w800),
        ),
      ),
      body: ListView(
        padding: const EdgeInsets.fromLTRB(16, 14, 16, 24),
        children: [
          ClipRRect(
            borderRadius: BorderRadius.circular(8),
            child: Image.asset(
              'assets/images/smart_home.png',
              height: 190,
              fit: BoxFit.cover,
            ),
          ),
          const SizedBox(height: 18),
          Text(
            english ? 'Home Devices' : '家庭设备',
            style: const TextStyle(fontSize: 17, fontWeight: FontWeight.w800),
          ),
          const SizedBox(height: 10),
          GridView.builder(
            shrinkWrap: true,
            physics: const NeverScrollableScrollPhysics(),
            gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(
              crossAxisCount: 2,
              crossAxisSpacing: 10,
              mainAxisSpacing: 10,
              childAspectRatio: 2.35,
            ),
            itemCount: actions.length,
            itemBuilder: (context, index) {
              final action = actions[index];
              return OutlinedButton.icon(
                onPressed: () => onCommand(action.$3, action.$4),
                icon: Icon(action.$1, color: action.$5),
                label: Text(
                  action.$2,
                  maxLines: 1,
                  overflow: TextOverflow.ellipsis,
                ),
                style: OutlinedButton.styleFrom(
                  foregroundColor: Colors.white,
                  alignment: Alignment.centerLeft,
                  side: BorderSide(color: action.$5.withValues(alpha: 0.62)),
                  shape: RoundedRectangleBorder(
                    borderRadius: BorderRadius.circular(8),
                  ),
                ),
              );
            },
          ),
        ],
      ),
    );
  }
}

class LegacyIndoorPatrolPage extends StatefulWidget {
  final bool isConnected;
  final ValueChanged<String> onCommand;
  final ValueNotifier<Map<String, dynamic>> statusNotifier;
  final bool isEnglish;

  const LegacyIndoorPatrolPage({
    super.key,
    required this.isConnected,
    required this.onCommand,
    required this.statusNotifier,
    this.isEnglish = false,
  });

  @override
  State<LegacyIndoorPatrolPage> createState() => _LegacyIndoorPatrolPageState();
}

class _LegacyIndoorPatrolPageState extends State<LegacyIndoorPatrolPage> {
  static const _rooms = ['客厅', '厨房', '主卧', '儿童房', '走廊'];
  static const _activeStates = {
    'started',
    'navigating',
    'inspecting',
    'retrying',
    'canceling',
  };

  void _togglePatrol(Map<String, dynamic> status) {
    final state = (status['state'] ?? 'idle').toString();
    widget.onCommand(_activeStates.contains(state) ? 'STOP' : 'START');
  }

  @override
  Widget build(BuildContext context) {
    final english = widget.isEnglish;
    return Scaffold(
      appBar: AppBar(
        title: Text(
          english ? 'Indoor Patrol' : '室内巡查',
          style: const TextStyle(fontWeight: FontWeight.w800),
        ),
      ),
      body: ValueListenableBuilder<Map<String, dynamic>>(
        valueListenable: widget.statusNotifier,
        builder: (context, status, _) => _buildBody(context, status),
      ),
    );
  }

  Widget _buildBody(BuildContext context, Map<String, dynamic> status) {
    final english = widget.isEnglish;
    const englishRooms = [
      'Living Room',
      'Kitchen',
      'Main Bedroom',
      'Child Room',
      'Hallway',
    ];
    String roomLabel(int index) =>
        english ? englishRooms[index] : _rooms[index];
    String localizedRoom(String room) {
      final index = _rooms.indexOf(room);
      return index >= 0 ? roomLabel(index) : room;
    }

    final state = (status['state'] ?? 'idle').toString();
    final running = _activeStates.contains(state);
    final currentRoom = (status['room'] ?? '').toString();
    final checkedRooms = (status['checked_rooms'] as List<dynamic>? ?? const [])
        .map((item) => item.toString())
        .toSet();
    final failedRooms = (status['failed_rooms'] as List<dynamic>? ?? const [])
        .map((item) => item.toString())
        .toSet();
    final summary = _statusSummary(status, localizedRoom(currentRoom), english);
    final problem = const {
      'not_configured',
      'nav_unavailable',
      'error',
      'failed',
      'room_failed',
      'completed_with_errors',
      'localization_lost',
    }.contains(state);

    return ListView(
      padding: const EdgeInsets.fromLTRB(16, 14, 16, 24),
      children: [
        ClipRRect(
          borderRadius: BorderRadius.circular(8),
          child: Image.asset(
            'assets/images/indoor_patrol.png',
            height: 180,
            fit: BoxFit.cover,
          ),
        ),
        const SizedBox(height: 14),
        Row(
          children: [
            Container(
              width: 9,
              height: 9,
              decoration: BoxDecoration(
                color: widget.isConnected
                    ? const Color(0xff22c55e)
                    : Colors.redAccent,
                shape: BoxShape.circle,
              ),
            ),
            const SizedBox(width: 8),
            Expanded(
              child: Text(
                english
                    ? (widget.isConnected ? 'Robot online' : 'Robot offline')
                    : (widget.isConnected ? '机器人在线' : '机器人未连接'),
              ),
            ),
            Flexible(
              child: Text(
                summary,
                textAlign: TextAlign.right,
                maxLines: 2,
                overflow: TextOverflow.ellipsis,
                style: TextStyle(
                  color: problem ? Colors.redAccent : Colors.white60,
                ),
              ),
            ),
          ],
        ),
        const SizedBox(height: 16),
        ...List.generate(_rooms.length, (index) {
          final room = _rooms[index];
          final done = checkedRooms.contains(room);
          final failed = failedRooms.contains(room);
          final active = running && currentRoom == room;
          return Container(
            margin: const EdgeInsets.only(bottom: 9),
            padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 13),
            decoration: BoxDecoration(
              color: const Color(0xff151b24),
              borderRadius: BorderRadius.circular(8),
              border: Border.all(
                color: active
                    ? Theme.of(context).colorScheme.primary
                    : const Color(0xff263241),
              ),
            ),
            child: Row(
              children: [
                Icon(
                  failed
                      ? Icons.error_outline_rounded
                      : done
                      ? Icons.check_circle_rounded
                      : active
                      ? Icons.radar_rounded
                      : Icons.radio_button_unchecked_rounded,
                  color: failed
                      ? Colors.redAccent
                      : done
                      ? const Color(0xff22c55e)
                      : active
                      ? Theme.of(context).colorScheme.primary
                      : Colors.white38,
                ),
                const SizedBox(width: 11),
                Expanded(
                  child: Text(
                    roomLabel(index),
                    style: const TextStyle(fontWeight: FontWeight.w700),
                  ),
                ),
                Text(
                  failed
                      ? (english ? 'Failed' : '未完成')
                      : done
                      ? (english ? 'Checked' : '已检查')
                      : active
                      ? state == 'inspecting'
                            ? (english ? 'Checking' : '检查中')
                            : state == 'retrying'
                            ? (english ? 'Retrying' : '重试中')
                            : (english ? 'En route' : '前往中')
                      : (english ? 'Pending' : '待检查'),
                  style: const TextStyle(color: Colors.white54, fontSize: 12),
                ),
              ],
            ),
          );
        }),
        const SizedBox(height: 6),
        FilledButton.icon(
          onPressed: widget.isConnected ? () => _togglePatrol(status) : null,
          icon: Icon(
            running ? Icons.stop_circle_rounded : Icons.play_arrow_rounded,
          ),
          label: Text(
            english
                ? (running ? 'End Patrol' : 'Start Home Patrol')
                : (running ? '结束本轮巡查' : '开始家庭巡查'),
          ),
          style: FilledButton.styleFrom(
            minimumSize: const Size.fromHeight(50),
            shape: RoundedRectangleBorder(
              borderRadius: BorderRadius.circular(8),
            ),
          ),
        ),
      ],
    );
  }

  String _statusSummary(
    Map<String, dynamic> status,
    String room,
    bool english,
  ) {
    final state = (status['state'] ?? 'idle').toString();
    if (!english) {
      return (status['message'] ?? '等待开始').toString();
    }
    return switch (state) {
      'started' => 'Patrol started',
      'navigating' => room.isEmpty ? 'Navigating' : 'Heading to $room',
      'inspecting' => room.isEmpty ? 'Inspecting' : 'Inspecting $room',
      'checked' => room.isEmpty ? 'Room checked' : '$room checked',
      'retrying' => room.isEmpty ? 'Retrying' : 'Retrying $room',
      'canceling' => 'Stopping patrol',
      'canceled' => 'Patrol stopped',
      'completed' => 'Patrol complete',
      'completed_with_errors' => 'Completed with unreachable rooms',
      'not_configured' => 'Room waypoints are not configured',
      'nav_unavailable' => 'Nav2 is not running',
      'failed' => 'Patrol failed',
      'error' => 'Patrol command error',
      _ => 'Ready',
    };
  }
}

class IndoorPatrolPage extends StatefulWidget {
  final bool isConnected;
  final ValueChanged<String> onCommand;
  final ValueNotifier<Map<String, dynamic>> statusNotifier;
  final AppSettingsController settings;
  final String mapBaseUrl;
  final List<PatrolWaypoint>? initialPath;
  final bool isEnglish;

  const IndoorPatrolPage({
    super.key,
    required this.isConnected,
    required this.onCommand,
    required this.statusNotifier,
    required this.settings,
    required this.mapBaseUrl,
    this.initialPath,
    this.isEnglish = false,
  });

  @override
  State<IndoorPatrolPage> createState() => _IndoorPatrolPageState();
}

class _IndoorPatrolPageState extends State<IndoorPatrolPage> {
  static const _defaultRooms = ['客厅', '厨房', '主卧', '儿童房', '走廊'];
  static const _englishRooms = [
    'Living Room',
    'Kitchen',
    'Main Bedroom',
    'Child Room',
    'Hallway',
  ];
  static const _activeStates = {
    'started',
    'navigating',
    'inspecting',
    'retrying',
    'canceling',
  };
  late List<PatrolWaypoint> _path;
  late Map<String, PatrolWaypoint> _defaultWaypoints;

  @override
  void initState() {
    super.initState();
    _path = List.of(widget.initialPath ?? widget.settings.customPatrolPath);
    _defaultWaypoints = Map.of(widget.settings.defaultRoomWaypoints);
  }

  Future<void> _editDefaultRoom(String room) async {
    final existing = _defaultWaypoints[room];
    final result = await Navigator.of(context).push<List<PatrolWaypoint>>(
      MaterialPageRoute(
        builder: (_) => PatrolPathEditorPage(
          mapBaseUrl: widget.mapBaseUrl,
          initialPath: existing == null ? const [] : [existing],
          roomName: room,
          isEnglish: widget.isEnglish,
        ),
      ),
    );
    if (!mounted || result == null || result.isEmpty) return;
    final waypoint = result.single;
    setState(() => _defaultWaypoints[room] = waypoint);
    await widget.settings.saveDefaultRoomWaypoint(waypoint);
    if (!mounted) return;
    if (widget.isConnected) {
      widget.onCommand(
        jsonEncode({'command': 'SET_ROOM', 'waypoint': waypoint.toMap()}),
      );
      ScaffoldMessenger.of(
        context,
      ).showSnackBar(SnackBar(content: Text('已发送$room巡查位置到机器人')));
    } else {
      ScaffoldMessenger.of(
        context,
      ).showSnackBar(SnackBar(content: Text('$room位置已保存在手机，连接机器人后请再次保存')));
    }
  }

  Future<void> _editPath() async {
    final result = await Navigator.of(context).push<List<PatrolWaypoint>>(
      MaterialPageRoute(
        builder: (_) => PatrolPathEditorPage(
          mapBaseUrl: widget.mapBaseUrl,
          initialPath: _path,
          isEnglish: widget.isEnglish,
        ),
      ),
    );
    if (!mounted || result == null) return;
    setState(() => _path = result);
    await widget.settings.saveCustomPatrolPath(result);
  }

  void _startDefaultPatrol() {
    if (_defaultWaypoints.isNotEmpty &&
        _defaultWaypoints.length < _defaultRooms.length) {
      final missing = _defaultRooms
          .where((room) => !_defaultWaypoints.containsKey(room))
          .join('、');
      ScaffoldMessenger.of(
        context,
      ).showSnackBar(SnackBar(content: Text('请先设置这些房间的位置：$missing')));
      return;
    }
    if (_defaultWaypoints.length == _defaultRooms.length) {
      for (final room in _defaultRooms) {
        widget.onCommand(
          jsonEncode({
            'command': 'SET_ROOM',
            'waypoint': _defaultWaypoints[room]!.toMap(),
          }),
        );
      }
    }
    widget.onCommand('START');
  }

  void _toggleCustomPatrol(Map<String, dynamic> status) {
    final state = (status['state'] ?? 'idle').toString();
    if (_activeStates.contains(state)) {
      widget.onCommand('STOP');
      return;
    }
    if (_path.isEmpty) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text(
            widget.isEnglish
                ? 'Add at least one patrol point first'
                : '请先在地图上添加至少一个巡查点',
          ),
        ),
      );
      return;
    }
    widget.onCommand(
      jsonEncode({
        'command': 'START_PATH',
        'waypoints': _path.map((item) => item.toMap()).toList(),
      }),
    );
  }

  @override
  Widget build(BuildContext context) {
    final english = widget.isEnglish;
    return Scaffold(
      appBar: AppBar(
        title: Text(
          english ? 'Indoor Patrol' : '室内巡查',
          style: const TextStyle(fontWeight: FontWeight.w800),
        ),
      ),
      body: ValueListenableBuilder<Map<String, dynamic>>(
        valueListenable: widget.statusNotifier,
        builder: (context, status, _) {
          final state = (status['state'] ?? 'idle').toString();
          final running = _activeStates.contains(state);
          final current = (status['room'] ?? '').toString();
          final checked =
              (status['checked_rooms'] as List<dynamic>? ?? const [])
                  .map((item) => item.toString())
                  .toSet();
          final failed = (status['failed_rooms'] as List<dynamic>? ?? const [])
              .map((item) => item.toString())
              .toSet();
          final problem = const {
            'not_configured',
            'nav_unavailable',
            'invalid_path',
            'error',
            'failed',
            'room_failed',
            'completed_with_errors',
            'localization_lost',
          }.contains(state);

          return Column(
            children: [
              Expanded(
                child: ListView(
                  padding: const EdgeInsets.fromLTRB(16, 14, 16, 12),
                  children: [
                    ClipRRect(
                      borderRadius: BorderRadius.circular(8),
                      child: Image.asset(
                        'assets/images/indoor_patrol.png',
                        height: 150,
                        fit: BoxFit.cover,
                      ),
                    ),
                    const SizedBox(height: 13),
                    Row(
                      children: [
                        Icon(
                          problem
                              ? Icons.error_outline_rounded
                              : widget.isConnected
                              ? Icons.cloud_done_rounded
                              : Icons.cloud_off_rounded,
                          color: problem
                              ? Colors.redAccent
                              : widget.isConnected
                              ? const Color(0xff22c55e)
                              : Colors.redAccent,
                        ),
                        const SizedBox(width: 9),
                        Expanded(
                          child: Text(
                            _statusSummary(status),
                            maxLines: 2,
                            overflow: TextOverflow.ellipsis,
                            style: TextStyle(
                              fontWeight: FontWeight.w700,
                              color: problem ? Colors.redAccent : null,
                            ),
                          ),
                        ),
                        Text(
                          english
                              ? '${_defaultWaypoints.length}/5 set · ${_path.length} custom'
                              : '默认已设置 ${_defaultWaypoints.length}/5 · 自定义 ${_path.length} 个',
                          style: const TextStyle(color: Colors.white54),
                        ),
                      ],
                    ),
                    const SizedBox(height: 14),
                    Text(
                      english ? 'Default Room Patrol' : '默认房间巡查',
                      style: const TextStyle(
                        fontSize: 16,
                        fontWeight: FontWeight.w800,
                      ),
                    ),
                    const SizedBox(height: 9),
                    ..._defaultRooms.asMap().entries.map((entry) {
                      final room = entry.value;
                      final isCurrent = running && current == room;
                      final isDone = checked.contains(room);
                      final isFailed = failed.contains(room);
                      final waypoint = _defaultWaypoints[room];
                      final isConfigured = waypoint != null;
                      final color = isFailed
                          ? Colors.redAccent
                          : isDone
                          ? const Color(0xff22c55e)
                          : isCurrent
                          ? Theme.of(context).colorScheme.primary
                          : isConfigured
                          ? Theme.of(context).colorScheme.primary
                          : Colors.white38;
                      return Padding(
                        padding: const EdgeInsets.only(bottom: 8),
                        child: Material(
                          color: const Color(0xff151b24),
                          borderRadius: BorderRadius.circular(8),
                          child: InkWell(
                            key: ValueKey('default_room_$room'),
                            borderRadius: BorderRadius.circular(8),
                            onTap: running
                                ? null
                                : () => _editDefaultRoom(room),
                            child: Container(
                              padding: const EdgeInsets.symmetric(
                                horizontal: 13,
                                vertical: 11,
                              ),
                              decoration: BoxDecoration(
                                borderRadius: BorderRadius.circular(8),
                                border: Border.all(
                                  color: isCurrent
                                      ? color
                                      : const Color(0xff263241),
                                ),
                              ),
                              child: Row(
                                children: [
                                  Icon(
                                    isFailed
                                        ? Icons.error_outline_rounded
                                        : isDone
                                        ? Icons.check_circle_rounded
                                        : isCurrent
                                        ? Icons.radar_rounded
                                        : isConfigured
                                        ? Icons.location_on_rounded
                                        : Icons.add_location_alt_outlined,
                                    color: color,
                                  ),
                                  const SizedBox(width: 11),
                                  Expanded(
                                    child: Column(
                                      crossAxisAlignment:
                                          CrossAxisAlignment.start,
                                      children: [
                                        Text(
                                          english
                                              ? _englishRooms[entry.key]
                                              : room,
                                          style: const TextStyle(
                                            fontWeight: FontWeight.w700,
                                          ),
                                        ),
                                        if (waypoint != null)
                                          Text(
                                            'x ${waypoint.x.toStringAsFixed(2)}  y ${waypoint.y.toStringAsFixed(2)}',
                                            style: const TextStyle(
                                              color: Colors.white38,
                                              fontSize: 12,
                                            ),
                                          ),
                                      ],
                                    ),
                                  ),
                                  Text(
                                    isFailed
                                        ? (english ? 'Failed' : '未到达')
                                        : isDone
                                        ? (english ? 'Checked' : '已检查')
                                        : isCurrent
                                        ? (english ? 'In progress' : '巡查中')
                                        : isConfigured
                                        ? (english ? 'Set' : '已设置')
                                        : (english ? 'Set point' : '点击设置'),
                                    style: TextStyle(
                                      color: isConfigured
                                          ? color
                                          : Colors.white54,
                                      fontSize: 12,
                                    ),
                                  ),
                                  const SizedBox(width: 5),
                                  const Icon(
                                    Icons.chevron_right_rounded,
                                    color: Colors.white30,
                                  ),
                                ],
                              ),
                            ),
                          ),
                        ),
                      );
                    }),
                    const SizedBox(height: 8),
                    Row(
                      children: [
                        Text(
                          english ? 'Custom Route' : '自定义路径',
                          style: const TextStyle(
                            fontSize: 16,
                            fontWeight: FontWeight.w800,
                          ),
                        ),
                        const Spacer(),
                        Text(
                          english
                              ? '${_path.length} points'
                              : '${_path.length} 个巡查点',
                          style: const TextStyle(color: Colors.white54),
                        ),
                      ],
                    ),
                    const SizedBox(height: 9),
                    if (_path.isEmpty)
                      Container(
                        padding: const EdgeInsets.symmetric(vertical: 28),
                        decoration: BoxDecoration(
                          color: const Color(0xff151b24),
                          border: Border.all(color: const Color(0xff263241)),
                          borderRadius: BorderRadius.circular(8),
                        ),
                        child: Column(
                          children: [
                            const Icon(
                              Icons.route_rounded,
                              size: 38,
                              color: Colors.white38,
                            ),
                            const SizedBox(height: 8),
                            Text(
                              english ? 'No custom route' : '还没有自定义巡查路径',
                              style: const TextStyle(color: Colors.white60),
                            ),
                          ],
                        ),
                      )
                    else
                      ..._path.asMap().entries.map((entry) {
                        final point = entry.value;
                        final isCurrent = running && current == point.name;
                        final isDone = checked.contains(point.name);
                        final isFailed = failed.contains(point.name);
                        final color = isFailed
                            ? Colors.redAccent
                            : isDone
                            ? const Color(0xff22c55e)
                            : isCurrent
                            ? Theme.of(context).colorScheme.primary
                            : Colors.white38;
                        return Container(
                          margin: const EdgeInsets.only(bottom: 8),
                          padding: const EdgeInsets.symmetric(
                            horizontal: 13,
                            vertical: 11,
                          ),
                          decoration: BoxDecoration(
                            color: const Color(0xff151b24),
                            borderRadius: BorderRadius.circular(8),
                            border: Border.all(
                              color: isCurrent
                                  ? color
                                  : const Color(0xff263241),
                            ),
                          ),
                          child: Row(
                            children: [
                              CircleAvatar(
                                radius: 15,
                                backgroundColor: color.withValues(alpha: 0.18),
                                child: Text(
                                  '${entry.key + 1}',
                                  style: TextStyle(
                                    color: color,
                                    fontWeight: FontWeight.w800,
                                  ),
                                ),
                              ),
                              const SizedBox(width: 11),
                              Expanded(
                                child: Column(
                                  crossAxisAlignment: CrossAxisAlignment.start,
                                  children: [
                                    Text(
                                      point.name,
                                      style: const TextStyle(
                                        fontWeight: FontWeight.w700,
                                      ),
                                    ),
                                    Text(
                                      'x ${point.x.toStringAsFixed(2)}  y ${point.y.toStringAsFixed(2)}',
                                      style: const TextStyle(
                                        color: Colors.white38,
                                        fontSize: 12,
                                      ),
                                    ),
                                  ],
                                ),
                              ),
                              Icon(
                                isFailed
                                    ? Icons.error_outline_rounded
                                    : isDone
                                    ? Icons.check_circle_rounded
                                    : isCurrent
                                    ? Icons.radar_rounded
                                    : Icons.more_horiz_rounded,
                                color: color,
                              ),
                            ],
                          ),
                        );
                      }),
                  ],
                ),
              ),
              SafeArea(
                top: false,
                child: Padding(
                  padding: const EdgeInsets.fromLTRB(16, 8, 16, 12),
                  child: running
                      ? FilledButton.icon(
                          onPressed: widget.isConnected
                              ? () => widget.onCommand('STOP')
                              : null,
                          icon: const Icon(Icons.stop_circle_rounded),
                          label: Text(english ? 'Stop Patrol' : '停止巡查'),
                          style: FilledButton.styleFrom(
                            minimumSize: const Size.fromHeight(50),
                            shape: RoundedRectangleBorder(
                              borderRadius: BorderRadius.circular(8),
                            ),
                          ),
                        )
                      : Column(
                          crossAxisAlignment: CrossAxisAlignment.stretch,
                          children: [
                            FilledButton.icon(
                              onPressed: widget.isConnected
                                  ? _startDefaultPatrol
                                  : null,
                              icon: const Icon(Icons.home_work_rounded),
                              label: Text(
                                english
                                    ? 'Start Default Room Patrol'
                                    : '开始默认房间巡查',
                              ),
                              style: FilledButton.styleFrom(
                                minimumSize: const Size.fromHeight(48),
                                shape: RoundedRectangleBorder(
                                  borderRadius: BorderRadius.circular(8),
                                ),
                              ),
                            ),
                            const SizedBox(height: 9),
                            Row(
                              children: [
                                Expanded(
                                  child: OutlinedButton.icon(
                                    onPressed: _editPath,
                                    icon: const Icon(
                                      Icons.edit_location_alt_rounded,
                                    ),
                                    label: Text(
                                      english ? 'Customize Route' : '自定义巡查路径',
                                    ),
                                    style: OutlinedButton.styleFrom(
                                      minimumSize: const Size(0, 48),
                                      shape: RoundedRectangleBorder(
                                        borderRadius: BorderRadius.circular(8),
                                      ),
                                    ),
                                  ),
                                ),
                                const SizedBox(width: 10),
                                Expanded(
                                  child: FilledButton.tonalIcon(
                                    onPressed: widget.isConnected
                                        ? () => _toggleCustomPatrol(status)
                                        : null,
                                    icon: const Icon(Icons.route_rounded),
                                    label: Text(
                                      english ? 'Route Patrol' : '路径巡查',
                                    ),
                                    style: FilledButton.styleFrom(
                                      minimumSize: const Size(0, 48),
                                      shape: RoundedRectangleBorder(
                                        borderRadius: BorderRadius.circular(8),
                                      ),
                                    ),
                                  ),
                                ),
                              ],
                            ),
                          ],
                        ),
                ),
              ),
            ],
          );
        },
      ),
    );
  }

  String _statusSummary(Map<String, dynamic> status) {
    final message = (status['message'] ?? '').toString();
    if (!widget.isEnglish && message.isNotEmpty) return message;
    final room = (status['room'] ?? '').toString();
    return switch ((status['state'] ?? 'idle').toString()) {
      'started' => 'Patrol started',
      'navigating' => room.isEmpty ? 'Navigating' : 'Heading to $room',
      'inspecting' => room.isEmpty ? 'Inspecting' : 'Inspecting $room',
      'retrying' => 'Retrying $room',
      'completed' => 'Patrol complete',
      'completed_with_errors' => 'Patrol complete with unreachable points',
      'nav_unavailable' => 'Nav2 is not running',
      'invalid_path' => 'The patrol route is invalid',
      'canceled' => 'Patrol stopped',
      _ => widget.isEnglish ? 'Ready' : '等待开始路径巡查',
    };
  }
}

class SafetyCenterPage extends StatelessWidget {
  final Map<String, dynamic>? latestAlert;
  final List<Map<String, dynamic>> alerts;
  final bool isEnglish;

  const SafetyCenterPage({
    super.key,
    required this.latestAlert,
    required this.alerts,
    this.isEnglish = false,
  });

  @override
  Widget build(BuildContext context) {
    final english = isEnglish;
    return Scaffold(
      appBar: AppBar(
        title: Text(
          english ? 'Safety Alerts' : '安全预警',
          style: const TextStyle(fontWeight: FontWeight.w800),
        ),
      ),
      body: ListView(
        padding: const EdgeInsets.fromLTRB(16, 14, 16, 24),
        children: [
          ClipRRect(
            borderRadius: BorderRadius.circular(8),
            child: Image.asset(
              'assets/images/safety_alert.png',
              height: 180,
              fit: BoxFit.cover,
            ),
          ),
          const SizedBox(height: 14),
          Container(
            padding: const EdgeInsets.all(15),
            decoration: BoxDecoration(
              color: latestAlert == null
                  ? const Color(0xff10261a)
                  : const Color(0xff2b1418),
              borderRadius: BorderRadius.circular(8),
              border: Border.all(
                color: latestAlert == null
                    ? const Color(0xff22c55e)
                    : Colors.redAccent,
              ),
            ),
            child: Row(
              children: [
                Icon(
                  latestAlert == null
                      ? Icons.verified_user_rounded
                      : Icons.warning_amber_rounded,
                  color: latestAlert == null
                      ? const Color(0xff22c55e)
                      : Colors.redAccent,
                  size: 30,
                ),
                const SizedBox(width: 12),
                Expanded(
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        latestAlert == null
                            ? (english ? 'Home is secure' : '家庭状态平稳')
                            : (english
                                  ? 'Alert requires attention'
                                  : '存在待关注预警'),
                        style: const TextStyle(
                          fontSize: 16,
                          fontWeight: FontWeight.w800,
                        ),
                      ),
                      const SizedBox(height: 3),
                      Text(
                        latestAlert == null
                            ? (english
                                  ? 'No unusual events received'
                                  : '当前没有收到异常事件')
                            : '${latestAlert!['message'] ?? latestAlert!['异常项'] ?? (english ? 'Please check your home' : '请检查家庭环境')}',
                        style: const TextStyle(color: Colors.white60),
                      ),
                    ],
                  ),
                ),
              ],
            ),
          ),
          const SizedBox(height: 18),
          Text(
            english ? 'Recent Events' : '最近事件',
            style: const TextStyle(fontSize: 17, fontWeight: FontWeight.w800),
          ),
          const SizedBox(height: 9),
          if (alerts.isEmpty)
            Padding(
              padding: const EdgeInsets.symmetric(vertical: 34),
              child: Column(
                children: [
                  const Icon(
                    Icons.notifications_none_rounded,
                    size: 44,
                    color: Colors.white24,
                  ),
                  const SizedBox(height: 8),
                  Text(
                    english ? 'No alert records' : '暂无预警记录',
                    style: const TextStyle(color: Colors.white38),
                  ),
                ],
              ),
            )
          else
            ...alerts.reversed.map(
              (alert) => ListTile(
                contentPadding: const EdgeInsets.symmetric(horizontal: 4),
                leading: const Icon(
                  Icons.warning_amber_rounded,
                  color: Colors.amber,
                ),
                title: Text(
                  '${alert['message'] ?? alert['异常项'] ?? (english ? 'Home alert' : '家庭异常')}',
                ),
                subtitle: Text(
                  '${alert['location'] ?? alert['位置'] ?? (english ? 'Home' : '家庭')} · ${alert['received_at'] ?? alert['timestamp'] ?? ''}',
                  maxLines: 1,
                  overflow: TextOverflow.ellipsis,
                ),
              ),
            ),
        ],
      ),
    );
  }
}

class CareRecordsPage extends StatelessWidget {
  final ValueNotifier<List<Map<String, String>>> dialogueNotifier;
  final bool isEnglish;

  const CareRecordsPage({
    super.key,
    required this.dialogueNotifier,
    this.isEnglish = false,
  });

  @override
  Widget build(BuildContext context) {
    final english = isEnglish;
    return Scaffold(
      appBar: AppBar(
        title: Text(
          english ? 'Care Records' : '看护记录',
          style: const TextStyle(fontWeight: FontWeight.w800),
        ),
      ),
      body: ValueListenableBuilder<List<Map<String, String>>>(
        valueListenable: dialogueNotifier,
        builder: (context, messages, _) {
          return ListView(
            padding: const EdgeInsets.fromLTRB(16, 14, 16, 24),
            children: [
              ClipRRect(
                borderRadius: BorderRadius.circular(8),
                child: Image.asset(
                  'assets/images/child_care.png',
                  height: 170,
                  fit: BoxFit.cover,
                ),
              ),
              const SizedBox(height: 16),
              Row(
                children: [
                  Expanded(
                    child: Text(
                      english ? 'On-site Conversations' : '现场对话记录',
                      maxLines: 1,
                      overflow: TextOverflow.ellipsis,
                      style: const TextStyle(
                        fontSize: 17,
                        fontWeight: FontWeight.w800,
                      ),
                    ),
                  ),
                  const SizedBox(width: 10),
                  Text(
                    english
                        ? '${messages.length} messages'
                        : '${messages.length} 条',
                    style: const TextStyle(color: Colors.white54),
                  ),
                ],
              ),
              const SizedBox(height: 8),
              if (messages.isEmpty)
                Padding(
                  padding: const EdgeInsets.symmetric(vertical: 38),
                  child: Center(
                    child: Text(
                      english ? 'No care conversations yet' : '还没有看护对话记录',
                      style: const TextStyle(color: Colors.white38),
                    ),
                  ),
                )
              else
                ...messages.reversed.map((message) {
                  final role = message['role'] ?? 'user';
                  final isParent = role == 'parent';
                  final isRobot = role == 'robot' || role == 'assistant';
                  final name = isParent
                      ? (english ? 'Parent' : '家长')
                      : isRobot
                      ? (english ? 'Robot' : '机器人')
                      : (english ? 'On site' : '现场');
                  final color = isParent
                      ? const Color(0xfff59e0b)
                      : isRobot
                      ? const Color(0xff38bdf8)
                      : const Color(0xff22c55e);
                  return Container(
                    margin: const EdgeInsets.only(bottom: 8),
                    padding: const EdgeInsets.all(13),
                    decoration: BoxDecoration(
                      color: const Color(0xff151b24),
                      borderRadius: BorderRadius.circular(8),
                      border: Border.all(color: color.withValues(alpha: 0.35)),
                    ),
                    child: Row(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        Icon(Icons.chat_bubble_rounded, color: color, size: 18),
                        const SizedBox(width: 10),
                        Expanded(
                          child: Column(
                            crossAxisAlignment: CrossAxisAlignment.start,
                            children: [
                              Text(
                                '$name · ${message['time'] ?? '--:--'}',
                                style: TextStyle(
                                  color: color,
                                  fontSize: 12,
                                  fontWeight: FontWeight.w800,
                                ),
                              ),
                              const SizedBox(height: 4),
                              Text(message['text'] ?? ''),
                            ],
                          ),
                        ),
                      ],
                    ),
                  );
                }),
            ],
          );
        },
      ),
    );
  }
}

class SmartHomeControlSheet extends StatelessWidget {
  final void Function(String command, String feedback) onCommand;
  final VoidCallback onEmergencyStop;

  const SmartHomeControlSheet({
    super.key,
    required this.onCommand,
    required this.onEmergencyStop,
  });

  @override
  Widget build(BuildContext context) {
    final actions = [
      (
        Icons.lightbulb_rounded,
        '打开灯光',
        const Color(0xff22c55e),
        () => onCommand('LIGHT_ON', '客厅灯已打开'),
      ),
      (
        Icons.lightbulb_outline_rounded,
        '关闭灯光',
        const Color(0xff94a3b8),
        () => onCommand('LIGHT_OFF', '客厅灯已关闭'),
      ),
      (
        Icons.air_rounded,
        '开启通风',
        const Color(0xff38bdf8),
        () => onCommand('FAN_ON', '室内通风已开启'),
      ),
      (
        Icons.air_outlined,
        '关闭通风',
        const Color(0xff94a3b8),
        () => onCommand('FAN_OFF', '室内通风已关闭'),
      ),
      (
        Icons.notifications_active_rounded,
        '声光预警',
        const Color(0xfff59e0b),
        () => onCommand('ALARM_ON', '声光预警已开启'),
      ),
      (
        Icons.notifications_off_rounded,
        '解除预警',
        const Color(0xffa78bfa),
        () => onCommand('ALARM_OFF', '声光预警已解除'),
      ),
      (
        Icons.power_settings_new_rounded,
        '全部关闭',
        const Color(0xfff97316),
        () => onCommand('ALL_OFF', '家庭联动设备已全部关闭'),
      ),
      (
        Icons.stop_circle_rounded,
        '立即停止',
        const Color(0xffef4444),
        onEmergencyStop,
      ),
    ];

    return SingleChildScrollView(
      padding: const EdgeInsets.fromLTRB(16, 10, 16, 20),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          Container(
            width: 42,
            height: 4,
            decoration: BoxDecoration(
              color: Colors.white24,
              borderRadius: BorderRadius.circular(2),
            ),
          ),
          const SizedBox(height: 12),
          Row(
            children: [
              const Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      '家居联动',
                      style: TextStyle(
                        color: Colors.white,
                        fontSize: 20,
                        fontWeight: FontWeight.w800,
                      ),
                    ),
                    SizedBox(height: 3),
                    Text(
                      '灯光、通风与家庭安全设备',
                      style: TextStyle(color: Colors.white54, fontSize: 12),
                    ),
                  ],
                ),
              ),
              IconButton(
                tooltip: '关闭',
                onPressed: () => Navigator.pop(context),
                icon: const Icon(Icons.close_rounded),
              ),
            ],
          ),
          const SizedBox(height: 12),
          ClipRRect(
            borderRadius: BorderRadius.circular(8),
            child: Image.asset(
              'assets/images/smart_home.png',
              width: double.infinity,
              height: 132,
              fit: BoxFit.cover,
            ),
          ),
          const SizedBox(height: 12),
          GridView.builder(
            shrinkWrap: true,
            physics: const NeverScrollableScrollPhysics(),
            gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(
              crossAxisCount: 2,
              crossAxisSpacing: 10,
              mainAxisSpacing: 10,
              childAspectRatio: 2.65,
            ),
            itemCount: actions.length,
            itemBuilder: (context, index) {
              final action = actions[index];
              return _buildAction(
                icon: action.$1,
                label: action.$2,
                color: action.$3,
                onTap: action.$4,
              );
            },
          ),
        ],
      ),
    );
  }

  Widget _buildAction({
    required IconData icon,
    required String label,
    required Color color,
    required VoidCallback onTap,
  }) {
    return Material(
      color: const Color(0xff151b24),
      borderRadius: BorderRadius.circular(8),
      child: InkWell(
        borderRadius: BorderRadius.circular(8),
        onTap: onTap,
        child: Container(
          padding: const EdgeInsets.symmetric(horizontal: 12),
          decoration: BoxDecoration(
            borderRadius: BorderRadius.circular(8),
            border: Border.all(color: color.withValues(alpha: 0.5)),
          ),
          child: Row(
            children: [
              Icon(icon, color: color, size: 21),
              const SizedBox(width: 9),
              Expanded(
                child: Text(
                  label,
                  maxLines: 1,
                  overflow: TextOverflow.ellipsis,
                  style: const TextStyle(
                    color: Colors.white,
                    fontSize: 13,
                    fontWeight: FontWeight.w700,
                  ),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}

class HomeMapState {
  final int width;
  final int height;
  final double resolution;
  final double originX;
  final double originY;
  final double originYaw;
  final int version;
  final bool robotAvailable;
  final double robotX;
  final double robotY;
  final double robotYaw;
  final String navigationState;

  const HomeMapState({
    required this.width,
    required this.height,
    required this.resolution,
    required this.originX,
    required this.originY,
    required this.originYaw,
    required this.version,
    required this.robotAvailable,
    required this.robotX,
    required this.robotY,
    required this.robotYaw,
    required this.navigationState,
  });

  bool get mapAvailable => width > 0 && height > 0 && resolution > 0;

  factory HomeMapState.fromJson(Map<String, dynamic> json) {
    final map = Map<String, dynamic>.from(json['map'] as Map? ?? const {});
    final origin = Map<String, dynamic>.from(map['origin'] as Map? ?? const {});
    final robot = Map<String, dynamic>.from(json['robot'] as Map? ?? const {});
    double number(Map<String, dynamic> source, String key) =>
        (source[key] as num?)?.toDouble() ?? 0;
    return HomeMapState(
      width: (map['width'] as num?)?.toInt() ?? 0,
      height: (map['height'] as num?)?.toInt() ?? 0,
      resolution: number(map, 'resolution'),
      originX: number(origin, 'x'),
      originY: number(origin, 'y'),
      originYaw: number(origin, 'yaw'),
      version: (map['version'] as num?)?.toInt() ?? 0,
      robotAvailable: robot['available'] == true,
      robotX: number(robot, 'x'),
      robotY: number(robot, 'y'),
      robotYaw: number(robot, 'yaw'),
      navigationState: robot['navigation_state']?.toString() ?? 'stopped',
    );
  }

  HomeMapState withoutRobot() => HomeMapState(
    width: width,
    height: height,
    resolution: resolution,
    originX: originX,
    originY: originY,
    originYaw: originYaw,
    version: version,
    robotAvailable: false,
    robotX: robotX,
    robotY: robotY,
    robotYaw: robotYaw,
    navigationState: navigationState,
  );

  Offset pixelToWorld(Offset pixel) {
    final localX = pixel.dx * resolution;
    final localY = (height - pixel.dy) * resolution;
    final c = cos(originYaw);
    final s = sin(originYaw);
    return Offset(
      originX + c * localX - s * localY,
      originY + s * localX + c * localY,
    );
  }

  Offset worldToPixel(Offset world) {
    final dx = world.dx - originX;
    final dy = world.dy - originY;
    final c = cos(originYaw);
    final s = sin(originYaw);
    final localX = c * dx + s * dy;
    final localY = -s * dx + c * dy;
    return Offset(localX / resolution, height - localY / resolution);
  }

  Rect fittedMapRect(Size size) {
    if (!mapAvailable || size.isEmpty) return Offset.zero & size;
    final scale = min(size.width / width, size.height / height);
    final fitted = Size(width * scale, height * scale);
    return Rect.fromLTWH(
      (size.width - fitted.width) / 2,
      (size.height - fitted.height) / 2,
      fitted.width,
      fitted.height,
    );
  }

  Offset worldToScreen(Offset world, Size size) {
    final rect = fittedMapRect(size);
    final pixel = worldToPixel(world);
    return Offset(
      rect.left + pixel.dx / width * rect.width,
      rect.top + pixel.dy / height * rect.height,
    );
  }

  Offset screenToWorld(Offset screen, Size size) {
    final rect = fittedMapRect(size);
    final clamped = Offset(
      screen.dx.clamp(rect.left, rect.right).toDouble(),
      screen.dy.clamp(rect.top, rect.bottom).toDouble(),
    );
    return pixelToWorld(
      Offset(
        (clamped.dx - rect.left) / rect.width * width,
        (clamped.dy - rect.top) / rect.height * height,
      ),
    );
  }
}

class PatrolPathEditorPage extends StatefulWidget {
  final String mapBaseUrl;
  final List<PatrolWaypoint> initialPath;
  final String? roomName;
  final bool isEnglish;

  const PatrolPathEditorPage({
    super.key,
    required this.mapBaseUrl,
    required this.initialPath,
    this.roomName,
    this.isEnglish = false,
  });

  @override
  State<PatrolPathEditorPage> createState() => _PatrolPathEditorPageState();
}

class _PatrolPathEditorPageState extends State<PatrolPathEditorPage> {
  final HttpClient _client = HttpClient()
    ..connectionTimeout = const Duration(seconds: 2);
  late List<PatrolWaypoint> _points;
  Timer? _timer;
  HomeMapState? _map;
  Uint8List? _mapBytes;
  Offset? _target;
  double _targetYaw = 0;
  bool _yawInitialized = false;
  bool _loading = false;
  bool _canPop = false;
  List<PatrolWaypoint>? _roomResult;
  String? _error;

  bool get _isRoomMode => widget.roomName != null;

  @override
  void initState() {
    super.initState();
    final initial = List<PatrolWaypoint>.of(widget.initialPath);
    _points = _isRoomMode ? [] : initial;
    if (initial.isNotEmpty) {
      _target = Offset(initial.last.x, initial.last.y);
      _targetYaw = initial.last.yaw;
      _yawInitialized = true;
    }
    _refresh();
    _timer = Timer.periodic(
      const Duration(milliseconds: 600),
      (_) => _refresh(),
    );
  }

  @override
  void dispose() {
    _timer?.cancel();
    _client.close(force: true);
    super.dispose();
  }

  Future<Uint8List> _getBytes(String path) async {
    final uri = Uri.parse('${widget.mapBaseUrl}$path');
    final request = await _client.getUrl(uri);
    request.headers.set(HttpHeaders.cacheControlHeader, 'no-cache');
    final response = await request.close().timeout(const Duration(seconds: 2));
    if (response.statusCode != 200) {
      throw HttpException('HTTP ${response.statusCode}', uri: uri);
    }
    return consolidateHttpClientResponseBytes(response);
  }

  Future<void> _refresh() async {
    if (_loading) return;
    _loading = true;
    try {
      final stateBytes = await _getBytes('/map_state.json');
      final state = HomeMapState.fromJson(
        Map<String, dynamic>.from(jsonDecode(utf8.decode(stateBytes)) as Map),
      );
      Uint8List? image = _mapBytes;
      if (state.mapAvailable &&
          (image == null || state.version != _map?.version)) {
        image = await _getBytes('/map.png?t=${state.version}');
      }
      if (!mounted) return;
      setState(() {
        _map = state;
        _mapBytes = image;
        if (state.mapAvailable) {
          _error = null;
          if (!_yawInitialized && state.robotAvailable) {
            _targetYaw = state.robotYaw;
            _yawInitialized = true;
          }
        } else {
          _error = widget.isEnglish
              ? 'Waiting for the robot /map topic'
              : '等待机器人发布 /map 地图话题';
        }
      });
    } catch (_) {
      if (mounted) {
        setState(
          () => _error = widget.isEnglish
              ? 'Map service is unavailable'
              : '地图服务暂不可用，请检查机器人连接',
        );
      }
    } finally {
      _loading = false;
    }
  }

  void _finish() {
    if (_canPop) return;
    setState(() => _canPop = true);
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (!mounted) return;
      Navigator.of(
        context,
      ).pop(_isRoomMode ? _roomResult : List<PatrolWaypoint>.of(_points));
    });
  }

  void _saveRoom() {
    final target = _target;
    final room = widget.roomName;
    if (target == null || room == null) return;
    _roomResult = [
      PatrolWaypoint(name: room, x: target.dx, y: target.dy, yaw: _targetYaw),
    ];
    _finish();
  }

  void _addPoint() {
    final target = _target;
    if (target == null) return;
    setState(() {
      _points.add(
        PatrolWaypoint(
          name: widget.isEnglish
              ? 'Patrol Point ${_points.length + 1}'
              : '巡查点 ${_points.length + 1}',
          x: target.dx,
          y: target.dy,
          yaw: _targetYaw,
        ),
      );
    });
  }

  @override
  Widget build(BuildContext context) {
    final english = widget.isEnglish;
    return PopScope<List<PatrolWaypoint>>(
      canPop: _canPop,
      onPopInvokedWithResult: (didPop, _) {
        if (!didPop) _finish();
      },
      child: Scaffold(
        appBar: AppBar(
          leading: IconButton(
            tooltip: english ? 'Back' : '返回',
            onPressed: _finish,
            icon: const Icon(Icons.arrow_back_rounded),
          ),
          title: Text(
            _isRoomMode
                ? (english
                      ? 'Set ${widget.roomName} Position'
                      : '设置${widget.roomName}位置')
                : (english ? 'Customize Patrol Route' : '自定义巡查路径'),
            style: const TextStyle(fontWeight: FontWeight.w800),
          ),
          actions: _isRoomMode
              ? const []
              : [
                  IconButton(
                    tooltip: english ? 'Undo point' : '撤销巡查点',
                    onPressed: _points.isEmpty
                        ? null
                        : () => setState(() => _points.removeLast()),
                    icon: const Icon(Icons.undo_rounded),
                  ),
                  IconButton(
                    tooltip: english ? 'Clear route' : '清空路径',
                    onPressed: _points.isEmpty
                        ? null
                        : () => setState(_points.clear),
                    icon: const Icon(Icons.delete_sweep_rounded),
                  ),
                ],
        ),
        body: SafeArea(
          child: Column(
            children: [
              Expanded(
                child: Padding(
                  padding: const EdgeInsets.fromLTRB(12, 12, 12, 8),
                  child: Container(
                    decoration: BoxDecoration(
                      color: Colors.black,
                      border: Border.all(color: const Color(0xff263241)),
                      borderRadius: BorderRadius.circular(8),
                    ),
                    child: ClipRRect(
                      borderRadius: BorderRadius.circular(8),
                      child: _buildMap(),
                    ),
                  ),
                ),
              ),
              Padding(
                padding: const EdgeInsets.fromLTRB(14, 4, 14, 12),
                child: Row(
                  children: [
                    Expanded(
                      child: Column(
                        crossAxisAlignment: CrossAxisAlignment.stretch,
                        children: [
                          Text(
                            _isRoomMode
                                ? (english
                                      ? 'Select ${widget.roomName} position'
                                      : '选择${widget.roomName}巡查位置')
                                : (english
                                      ? '${_points.length} patrol points'
                                      : '已添加 ${_points.length} 个巡查点'),
                            style: const TextStyle(fontWeight: FontWeight.w800),
                          ),
                          const SizedBox(height: 8),
                          FilledButton.icon(
                            onPressed:
                                _target == null || _map?.mapAvailable != true
                                ? null
                                : (_isRoomMode ? _saveRoom : _addPoint),
                            icon: Icon(
                              _isRoomMode
                                  ? Icons.save_rounded
                                  : Icons.add_location_alt_rounded,
                            ),
                            label: Text(
                              _isRoomMode
                                  ? (english
                                        ? 'Save as ${widget.roomName}'
                                        : '保存为${widget.roomName}')
                                  : (english ? 'Add Patrol Point' : '添加巡查点'),
                            ),
                            style: FilledButton.styleFrom(
                              minimumSize: const Size(0, 50),
                              shape: RoundedRectangleBorder(
                                borderRadius: BorderRadius.circular(8),
                              ),
                            ),
                          ),
                        ],
                      ),
                    ),
                    const SizedBox(width: 16),
                    OrientationJoystick(
                      yaw: _targetYaw - (_map?.originYaw ?? 0),
                      onChanged: (relativeYaw) => setState(() {
                        _targetYaw = relativeYaw + (_map?.originYaw ?? 0);
                        _yawInitialized = true;
                      }),
                    ),
                  ],
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildMap() {
    final map = _map;
    if (_mapBytes == null || map == null || !map.mapAvailable) {
      return Center(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            if (_error == null)
              const CircularProgressIndicator()
            else
              const Icon(Icons.map_outlined, size: 52, color: Colors.white38),
            const SizedBox(height: 10),
            Text(
              _error ?? (widget.isEnglish ? 'Loading home map' : '正在加载家庭地图'),
              textAlign: TextAlign.center,
              style: const TextStyle(color: Colors.white60),
            ),
          ],
        ),
      );
    }

    return LayoutBuilder(
      builder: (context, constraints) {
        final size = constraints.biggest;
        final targetScreen = _target == null
            ? null
            : map.worldToScreen(_target!, size);
        return GestureDetector(
          behavior: HitTestBehavior.opaque,
          onTapDown: (details) => setState(
            () => _target = map.screenToWorld(details.localPosition, size),
          ),
          onPanUpdate: (details) => setState(
            () => _target = map.screenToWorld(details.localPosition, size),
          ),
          child: Stack(
            fit: StackFit.expand,
            children: [
              Image.memory(
                _mapBytes!,
                fit: BoxFit.contain,
                gaplessPlayback: true,
              ),
              CustomPaint(
                painter: PatrolRoutePainter(
                  map: map,
                  size: size,
                  points: _points,
                ),
              ),
              if (map.robotAvailable)
                _mapMarker(
                  map.worldToScreen(Offset(map.robotX, map.robotY), size),
                  map.robotYaw - map.originYaw,
                  const Color(0xff22c55e),
                  Icons.smart_toy_rounded,
                  34,
                ),
              if (targetScreen != null)
                CustomPaint(
                  painter: TargetHeadingPainter(
                    center: targetScreen,
                    relativeYaw: _targetYaw - map.originYaw,
                    color: Theme.of(context).colorScheme.primary,
                  ),
                ),
              if (targetScreen != null)
                Positioned(
                  left: targetScreen.dx - 22,
                  top: targetScreen.dy - 22,
                  child: const Icon(
                    Icons.ads_click_rounded,
                    color: Colors.white,
                    size: 44,
                    shadows: [Shadow(color: Colors.black, blurRadius: 6)],
                  ),
                ),
            ],
          ),
        );
      },
    );
  }

  Widget _mapMarker(
    Offset center,
    double relativeYaw,
    Color color,
    IconData icon,
    double size,
  ) {
    return Positioned(
      left: center.dx - size / 2,
      top: center.dy - size / 2,
      child: Transform.rotate(
        angle: -relativeYaw + pi / 2,
        child: Icon(
          icon,
          size: size,
          color: color,
          shadows: const [Shadow(color: Colors.black, blurRadius: 5)],
        ),
      ),
    );
  }
}

class TargetHeadingPainter extends CustomPainter {
  final Offset center;
  final double relativeYaw;
  final Color color;

  TargetHeadingPainter({
    required this.center,
    required this.relativeYaw,
    required this.color,
  });

  @override
  void paint(Canvas canvas, Size size) {
    final direction = Offset(cos(relativeYaw), -sin(relativeYaw));
    final start = center + direction * 20;
    final end = center + direction * 74;
    final paint = Paint()
      ..color = color
      ..strokeWidth = 7
      ..strokeCap = StrokeCap.round
      ..style = PaintingStyle.stroke;
    canvas.drawLine(start, end, paint);

    final angle = atan2(direction.dy, direction.dx);
    final arrow = Path()
      ..moveTo(end.dx, end.dy)
      ..lineTo(end.dx - 19 * cos(angle - 0.55), end.dy - 19 * sin(angle - 0.55))
      ..moveTo(end.dx, end.dy)
      ..lineTo(
        end.dx - 19 * cos(angle + 0.55),
        end.dy - 19 * sin(angle + 0.55),
      );
    canvas.drawPath(arrow, paint);
  }

  @override
  bool shouldRepaint(covariant TargetHeadingPainter oldDelegate) =>
      oldDelegate.center != center ||
      oldDelegate.relativeYaw != relativeYaw ||
      oldDelegate.color != color;
}

class OrientationJoystick extends StatelessWidget {
  final double yaw;
  final ValueChanged<double> onChanged;

  const OrientationJoystick({
    super.key,
    required this.yaw,
    required this.onChanged,
  });

  @override
  Widget build(BuildContext context) {
    const diameter = 112.0;
    void update(Offset local) {
      final vector = local - const Offset(diameter / 2, diameter / 2);
      if (vector.distance < 8) return;
      onChanged(atan2(-vector.dy, vector.dx));
    }

    final knob = Offset(cos(yaw), -sin(yaw)) * 35 + const Offset(56, 56);
    return Tooltip(
      message: '拖动设置机器人朝向',
      child: GestureDetector(
        onTapDown: (details) => update(details.localPosition),
        onPanUpdate: (details) => update(details.localPosition),
        child: SizedBox(
          width: diameter,
          height: diameter,
          child: Stack(
            children: [
              Container(
                decoration: BoxDecoration(
                  shape: BoxShape.circle,
                  color: const Color(0xff151b24),
                  border: Border.all(color: const Color(0xff3b4c61), width: 2),
                ),
                child: const Center(
                  child: Icon(
                    Icons.explore_rounded,
                    color: Colors.white30,
                    size: 34,
                  ),
                ),
              ),
              Positioned(
                left: knob.dx - 13,
                top: knob.dy - 13,
                child: Container(
                  width: 26,
                  height: 26,
                  decoration: BoxDecoration(
                    color: Theme.of(context).colorScheme.primary,
                    shape: BoxShape.circle,
                    border: Border.all(color: Colors.white, width: 2),
                  ),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}

class PatrolRoutePainter extends CustomPainter {
  final HomeMapState map;
  final Size size;
  final List<PatrolWaypoint> points;

  PatrolRoutePainter({
    required this.map,
    required this.size,
    required this.points,
  });

  @override
  void paint(Canvas canvas, Size canvasSize) {
    if (points.isEmpty) return;
    final positions = points
        .map((point) => map.worldToScreen(Offset(point.x, point.y), size))
        .toList();
    final line = Paint()
      ..color = const Color(0xfff59e0b).withValues(alpha: 0.8)
      ..strokeWidth = 3
      ..style = PaintingStyle.stroke;
    if (positions.length > 1) {
      final path = Path()..moveTo(positions.first.dx, positions.first.dy);
      for (final position in positions.skip(1)) {
        path.lineTo(position.dx, position.dy);
      }
      canvas.drawPath(path, line);
    }
    for (var index = 0; index < positions.length; index++) {
      final position = positions[index];
      canvas.drawCircle(position, 13, Paint()..color = const Color(0xfff59e0b));
      final text = TextPainter(
        text: TextSpan(
          text: '${index + 1}',
          style: const TextStyle(
            color: Colors.black,
            fontSize: 12,
            fontWeight: FontWeight.w900,
          ),
        ),
        textDirection: TextDirection.ltr,
      )..layout();
      text.paint(canvas, position - Offset(text.width / 2, text.height / 2));
    }
  }

  @override
  bool shouldRepaint(covariant PatrolRoutePainter oldDelegate) => true;
}

class SlamMapPage extends StatefulWidget {
  final bool isConnected;
  final bool isEnglish;
  final String connectionText;
  final String mapBaseUrl;
  final AppSettingsController settings;
  final ValueChanged<String> onNavigate;
  final void Function(String room, PatrolWaypoint waypoint) onSaveRoom;
  final ValueChanged<String> onStartNavigation;
  final ValueNotifier<Map<String, dynamic>> navigationStatusNotifier;
  final ValueNotifier<Map<String, dynamic>> patrolStatusNotifier;

  const SlamMapPage({
    super.key,
    required this.isConnected,
    this.isEnglish = false,
    required this.connectionText,
    required this.mapBaseUrl,
    required this.settings,
    required this.onNavigate,
    required this.onSaveRoom,
    required this.onStartNavigation,
    required this.navigationStatusNotifier,
    required this.patrolStatusNotifier,
  });

  @override
  State<SlamMapPage> createState() => _SlamMapPageState();
}

class _SlamMapPageState extends State<SlamMapPage> {
  static const _rooms = ['客厅', '厨房', '主卧', '儿童房', '走廊'];
  static const _englishRooms = [
    'Living Room',
    'Kitchen',
    'Main Bedroom',
    'Child Room',
    'Hallway',
  ];
  String get mapImageUrl => '${widget.mapBaseUrl}/map.png';
  Timer? _timer;
  final HttpClient _httpClient = HttpClient()
    ..connectionTimeout = const Duration(seconds: 1);
  Uint8List? _mapBytes;
  HomeMapState? _mapState;
  bool _mapLoading = false;
  late String _statusText;
  late String _detailText;
  late Map<String, PatrolWaypoint> _roomWaypoints;
  String _arrivalMessage = '';
  String _lastPatrolEvent = '';
  String? _pendingNavigationRoom;

  @override
  void initState() {
    super.initState();
    _statusText = widget.isEnglish ? 'Loading home map' : '正在获取家庭地图';
    _detailText = widget.isEnglish
        ? 'The indoor map appears when the robot connects'
        : '连接机器人后将自动显示室内地图';
    _roomWaypoints = Map.of(widget.settings.defaultRoomWaypoints);
    widget.patrolStatusNotifier.addListener(_handlePatrolStatus);
    widget.navigationStatusNotifier.addListener(_handleNavigationStatus);
    WidgetsBinding.instance.addPostFrameCallback((_) => _handlePatrolStatus());
    _loadMap();
    _timer = Timer.periodic(
      const Duration(milliseconds: 900),
      (_) => _loadMap(),
    );
  }

  @override
  void dispose() {
    _timer?.cancel();
    widget.patrolStatusNotifier.removeListener(_handlePatrolStatus);
    widget.navigationStatusNotifier.removeListener(_handleNavigationStatus);
    _httpClient.close(force: true);
    super.dispose();
  }

  void _handlePatrolStatus() {
    if (!mounted) return;
    final status = widget.patrolStatusNotifier.value;
    final state = (status['state'] ?? '').toString();
    final room = (status['room'] ?? '').toString().trim();
    final message = (status['message'] ?? '').toString().trim();
    final timestamp = (status['timestamp'] ?? '').toString();
    final signature = '$state|$room|$message|$timestamp';
    if (signature == _lastPatrolEvent) return;
    _lastPatrolEvent = signature;

    String? notice;
    Color noticeColor = const Color(0xff15803d);
    if (state == 'inspecting') {
      notice = widget.isEnglish
          ? 'Arrived at ${_roomLabel(room)}'
          : '已到达${_roomLabel(room)}';
      HapticFeedback.mediumImpact();
    } else if (state == 'checked') {
      notice = widget.isEnglish
          ? '${_roomLabel(room)} check complete'
          : '${_roomLabel(room)}检查完成';
    } else if (state == 'room_failed' ||
        state == 'failed' ||
        state == 'localization_lost') {
      notice = message.isEmpty
          ? (widget.isEnglish ? 'Navigation failed' : '导航未能到达目标点')
          : message;
      noticeColor = const Color(0xffb91c1c);
      HapticFeedback.heavyImpact();
    }
    if (notice == null) return;
    setState(() => _arrivalMessage = notice!);
    ScaffoldMessenger.of(context)
      ..hideCurrentSnackBar()
      ..showSnackBar(
        SnackBar(
          content: Row(
            children: [
              Icon(
                noticeColor == const Color(0xff15803d)
                    ? Icons.check_circle_rounded
                    : Icons.error_rounded,
                color: Colors.white,
              ),
              const SizedBox(width: 10),
              Expanded(
                child: Text(
                  notice,
                  style: const TextStyle(fontWeight: FontWeight.w800),
                ),
              ),
            ],
          ),
          backgroundColor: noticeColor,
          duration: const Duration(seconds: 4),
        ),
      );
  }

  void _handleNavigationStatus() {
    if (!mounted || _pendingNavigationRoom == null) return;
    final state = (widget.navigationStatusNotifier.value['state'] ?? '')
        .toString();
    if (state != 'running') {
      if (state == 'localization_failed' ||
          state == 'navigation_unavailable' ||
          state == 'error' ||
          state == 'stale') {
        _pendingNavigationRoom = null;
        final message = (widget.navigationStatusNotifier.value['message'] ?? '')
            .toString();
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: Text(message.isEmpty ? '定位未完成，已取消排队的导航任务' : message),
            backgroundColor: Colors.redAccent,
          ),
        );
      }
      return;
    }
    final room = _pendingNavigationRoom!;
    _pendingNavigationRoom = null;
    _sendRoomNavigation(room, automatically: true);
  }

  Future<void> _loadMap() async {
    if (_mapLoading) return;
    _mapLoading = true;
    try {
      final stateRequest = await _httpClient.getUrl(
        Uri.parse('${widget.mapBaseUrl}/map_state.json'),
      );
      stateRequest.headers.set(HttpHeaders.cacheControlHeader, 'no-cache');
      final stateResponse = await stateRequest.close().timeout(
        const Duration(seconds: 2),
      );
      final stateBytes = await consolidateHttpClientResponseBytes(
        stateResponse,
      );
      if (stateResponse.statusCode != 200) {
        throw HttpException('Map state HTTP ${stateResponse.statusCode}');
      }
      final mapState = HomeMapState.fromJson(
        Map<String, dynamic>.from(jsonDecode(utf8.decode(stateBytes)) as Map),
      );
      var bytes = _mapBytes;
      if (mapState.mapAvailable &&
          (bytes == null || mapState.version != _mapState?.version)) {
        final request = await _httpClient.getUrl(
          Uri.parse('$mapImageUrl?t=${mapState.version}'),
        );
        request.headers.set(HttpHeaders.cacheControlHeader, 'no-cache');
        final response = await request.close().timeout(
          const Duration(seconds: 2),
        );
        bytes = await consolidateHttpClientResponseBytes(response);
        if (response.statusCode != 200 || bytes.isEmpty) {
          throw HttpException('Map image HTTP ${response.statusCode}');
        }
      }
      if (!mounted) return;
      if (mapState.mapAvailable && bytes != null && bytes.isNotEmpty) {
        setState(() {
          _mapBytes = bytes;
          _mapState = mapState;
          _statusText = widget.isEnglish ? 'Home map synchronized' : '家庭地图已同步';
          _detailText = widget.isEnglish
              ? (mapState.robotAvailable
                    ? 'Robot position is live'
                    : mapState.navigationState == 'stopped'
                    ? 'Tap Start Navigation to show the robot position'
                    : 'Navigation is starting; waiting for AMCL localization')
              : (mapState.robotAvailable
                    ? '机器人位置实时更新中'
                    : mapState.navigationState == 'stopped'
                    ? '请先点击“启动导航”，随后显示机器人位置'
                    : '导航正在启动，正在等待 AMCL 完成定位');
        });
      } else {
        setState(() {
          final waitingToStart = mapState.navigationState == 'stopped';
          _statusText = widget.isEnglish
              ? (waitingToStart
                    ? 'Navigation has not started'
                    : 'Map is loading')
              : (waitingToStart ? '导航尚未启动' : '家庭地图正在加载');
          _detailText = widget.isEnglish
              ? (waitingToStart
                    ? 'Tap Start Navigation; the map will appear after Nav2 loads'
                    : 'Please wait for Nav2 and AMCL localization')
              : (waitingToStart
                    ? '请点击“启动导航”，Nav2 加载后地图会自动显示'
                    : '请等待 Nav2 与 AMCL 完成定位');
        });
      }
    } catch (e) {
      if (!mounted) return;
      setState(() {
        _mapState = _mapState?.withoutRobot();
        _statusText = widget.isEnglish
            ? 'Map is temporarily unavailable'
            : '暂时无法显示地图';
        _detailText = widget.isEnglish
            ? 'Check that the robot main program is running'
            : '请确认机器人一键主程序仍在运行';
      });
    } finally {
      _mapLoading = false;
    }
  }

  String _roomLabel(String room) {
    final index = _rooms.indexOf(room);
    return widget.isEnglish && index >= 0 ? _englishRooms[index] : room;
  }

  Future<void> _editRoom(String room) async {
    final existing = _roomWaypoints[room];
    final result = await Navigator.of(context).push<List<PatrolWaypoint>>(
      MaterialPageRoute(
        builder: (_) => PatrolPathEditorPage(
          mapBaseUrl: widget.mapBaseUrl,
          initialPath: existing == null ? const [] : [existing],
          roomName: room,
          isEnglish: widget.isEnglish,
        ),
      ),
    );
    if (!mounted || result == null || result.isEmpty) return;
    final waypoint = result.first;
    setState(() => _roomWaypoints[room] = waypoint);
    await widget.settings.saveDefaultRoomWaypoint(waypoint);
    widget.onSaveRoom(room, waypoint);
    if (!mounted) return;
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(
          widget.isEnglish
              ? '${_roomLabel(room)} target saved'
              : '已保存${_roomLabel(room)}目标点',
        ),
      ),
    );
  }

  void _navigateRoom(String room) {
    if (!_roomWaypoints.containsKey(room)) {
      ScaffoldMessenger.of(
        context,
      ).showSnackBar(SnackBar(content: Text('请先设置${_roomLabel(room)}目标点')));
      return;
    }
    final navigationState =
        (widget.navigationStatusNotifier.value['state'] ?? '').toString();
    if (navigationState == 'starting' || navigationState == 'localizing') {
      _pendingNavigationRoom = room;
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text(
            widget.isEnglish
                ? '${_roomLabel(room)} queued; it will start when navigation is ready'
                : '已排队前往${_roomLabel(room)}，导航就绪后自动发送',
          ),
        ),
      );
      return;
    }
    if (navigationState != 'running') {
      final localizationFailed = navigationState == 'localization_failed';
      final unavailable =
          navigationState == 'navigation_unavailable' ||
          navigationState == 'stale';
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text(
            widget.isEnglish
                ? 'Start the navigation system before sending a room target'
                : localizationFailed
                ? 'AMCL 尚未定位成功，请把机器人放回建图原点后点击“重新定位”'
                : unavailable
                ? '导航后台已停止响应，请停止后重新启动导航'
                : '请先点击“启动导航系统”，就绪后再发送房间目标',
          ),
          backgroundColor: localizationFailed || unavailable
              ? Colors.redAccent
              : null,
        ),
      );
      return;
    }
    _sendRoomNavigation(room);
  }

  void _sendRoomNavigation(String room, {bool automatically = false}) {
    widget.onNavigate(room);
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(
          widget.isEnglish
              ? 'Navigation to ${_roomLabel(room)} ${automatically ? 'started automatically' : 'sent'}'
              : '${automatically ? '导航已就绪，自动发送' : '已发送'}前往${_roomLabel(room)}的任务',
        ),
      ),
    );
  }

  void _startNavigation() {
    widget.onStartNavigation('');
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(
          widget.isEnglish
              ? 'Starting navigation and locating at the mapping origin'
              : '正在启动导航，并自动定位到建图原点',
        ),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    final english = widget.isEnglish;
    final statusColor = widget.isConnected
        ? const Color(0xff22c55e)
        : Colors.redAccent;
    return Scaffold(
      backgroundColor: const Color(0xff0d1117),
      appBar: AppBar(
        title: Text(
          english ? 'Home Map' : '家庭地图',
          style: const TextStyle(fontWeight: FontWeight.w800),
        ),
        backgroundColor: const Color(0xff10151d),
        elevation: 0,
        actions: [
          IconButton(
            tooltip: english ? 'Refresh map' : '刷新地图',
            onPressed: _loadMap,
            icon: const Icon(Icons.refresh_rounded),
          ),
        ],
      ),
      body: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          children: [
            Container(
              width: double.infinity,
              padding: const EdgeInsets.all(14),
              decoration: BoxDecoration(
                color: const Color(0xff151b24),
                borderRadius: BorderRadius.circular(8),
                border: Border.all(color: const Color(0xff263241)),
              ),
              child: Row(
                children: [
                  Icon(Icons.router_rounded, color: statusColor),
                  const SizedBox(width: 10),
                  Expanded(
                    child: Text(
                      english
                          ? 'Robot: ${widget.isConnected ? 'Online' : 'Offline'}'
                          : '机器人状态：${widget.connectionText}',
                      style: const TextStyle(
                        color: Colors.white,
                        fontWeight: FontWeight.w700,
                      ),
                    ),
                  ),
                ],
              ),
            ),
            const SizedBox(height: 14),
            Expanded(
              child: Container(
                width: double.infinity,
                decoration: BoxDecoration(
                  color: Colors.black,
                  borderRadius: BorderRadius.circular(8),
                  border: Border.all(color: const Color(0xff263241)),
                ),
                child: ClipRRect(
                  borderRadius: BorderRadius.circular(8),
                  child: Stack(
                    fit: StackFit.expand,
                    children: [
                      if (_mapBytes != null && _mapState != null)
                        LayoutBuilder(
                          builder: (context, constraints) {
                            final state = _mapState!;
                            final robot = state.robotAvailable
                                ? state.worldToScreen(
                                    Offset(state.robotX, state.robotY),
                                    constraints.biggest,
                                  )
                                : null;
                            return Stack(
                              fit: StackFit.expand,
                              children: [
                                Image.memory(
                                  _mapBytes!,
                                  fit: BoxFit.contain,
                                  gaplessPlayback: true,
                                ),
                                if (robot != null)
                                  Positioned(
                                    left: robot.dx - 20,
                                    top: robot.dy - 20,
                                    child: Transform.rotate(
                                      angle:
                                          -(state.robotYaw - state.originYaw) +
                                          pi / 2,
                                      child: const Icon(
                                        Icons.navigation_rounded,
                                        size: 40,
                                        color: Color(0xff22c55e),
                                        shadows: [
                                          Shadow(
                                            color: Colors.black,
                                            blurRadius: 6,
                                          ),
                                        ],
                                      ),
                                    ),
                                  ),
                              ],
                            );
                          },
                        )
                      else
                        Center(
                          child: Column(
                            mainAxisSize: MainAxisSize.min,
                            children: [
                              Icon(
                                Icons.map_rounded,
                                size: 54,
                                color: const Color(
                                  0xff38bdf8,
                                ).withValues(alpha: 0.72),
                              ),
                              const SizedBox(height: 10),
                              Text(
                                english ? 'Loading Home Map' : '正在获取家庭地图',
                                style: const TextStyle(
                                  color: Colors.white,
                                  fontSize: 18,
                                  fontWeight: FontWeight.w800,
                                ),
                              ),
                              const SizedBox(height: 6),
                              Text(
                                _detailText,
                                textAlign: TextAlign.center,
                                style: TextStyle(
                                  color: Colors.white.withValues(alpha: 0.52),
                                  fontSize: 12,
                                ),
                              ),
                            ],
                          ),
                        ),
                      Positioned(
                        left: 10,
                        right: 10,
                        bottom: 10,
                        child: Container(
                          padding: const EdgeInsets.symmetric(
                            horizontal: 12,
                            vertical: 8,
                          ),
                          decoration: BoxDecoration(
                            color: const Color(0xcc0d1117),
                            borderRadius: BorderRadius.circular(8),
                            border: Border.all(color: const Color(0xff263241)),
                          ),
                          child: Text(
                            '$_statusText · $_detailText',
                            maxLines: 2,
                            overflow: TextOverflow.ellipsis,
                            style: const TextStyle(
                              color: Colors.white,
                              fontSize: 12,
                              fontWeight: FontWeight.w700,
                            ),
                          ),
                        ),
                      ),
                    ],
                  ),
                ),
              ),
            ),
            const SizedBox(height: 14),
            ValueListenableBuilder<Map<String, dynamic>>(
              valueListenable: widget.navigationStatusNotifier,
              builder: (context, status, _) {
                final state = (status['state'] ?? 'stopped').toString();
                final message = (status['message'] ?? '').toString();
                final running = state == 'running';
                final starting = state == 'starting' || state == 'localizing';
                return Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    SizedBox(
                      width: double.infinity,
                      child: FilledButton.icon(
                        onPressed: starting ? null : _startNavigation,
                        icon: Icon(
                          running
                              ? Icons.my_location_rounded
                              : Icons.power_settings_new_rounded,
                        ),
                        label: Text(
                          running
                              ? (english
                                    ? 'Relocate to Mapping Origin'
                                    : '重新定位到建图原点')
                              : starting
                              ? (english ? 'Starting...' : '正在启动...')
                              : (english ? 'Start Navigation' : '启动导航系统'),
                        ),
                      ),
                    ),
                    if (message.isNotEmpty) ...[
                      const SizedBox(height: 6),
                      Text(
                        message,
                        maxLines: 2,
                        overflow: TextOverflow.ellipsis,
                        style: TextStyle(
                          color:
                              state == 'error' ||
                                  state == 'stale' ||
                                  state == 'localization_failed' ||
                                  state == 'navigation_unavailable' ||
                                  state == 'room_not_configured'
                              ? Colors.redAccent
                              : running
                              ? const Color(0xff22c55e)
                              : Colors.white54,
                          fontSize: 12,
                          fontWeight: FontWeight.w600,
                        ),
                      ),
                    ],
                    if (_arrivalMessage.isNotEmpty) ...[
                      const SizedBox(height: 8),
                      Container(
                        width: double.infinity,
                        padding: const EdgeInsets.symmetric(
                          horizontal: 12,
                          vertical: 9,
                        ),
                        decoration: BoxDecoration(
                          color: const Color(
                            0xff15803d,
                          ).withValues(alpha: 0.24),
                          borderRadius: BorderRadius.circular(8),
                          border: Border.all(color: const Color(0xff22c55e)),
                        ),
                        child: Row(
                          children: [
                            const Icon(
                              Icons.check_circle_rounded,
                              size: 18,
                              color: Color(0xff22c55e),
                            ),
                            const SizedBox(width: 8),
                            Expanded(
                              child: Text(
                                _arrivalMessage,
                                style: const TextStyle(
                                  color: Colors.white,
                                  fontWeight: FontWeight.w800,
                                ),
                              ),
                            ),
                          ],
                        ),
                      ),
                    ],
                  ],
                );
              },
            ),
            const SizedBox(height: 9),
            Align(
              alignment: Alignment.centerLeft,
              child: Text(
                english
                    ? 'Room Targets · tap a room to set its position'
                    : '房间目标点 · 点房间设置位置，点箭头开始导航',
                style: const TextStyle(
                  fontSize: 14,
                  fontWeight: FontWeight.w800,
                ),
              ),
            ),
            const SizedBox(height: 7),
            SizedBox(
              height: 72,
              child: ListView.separated(
                scrollDirection: Axis.horizontal,
                itemCount: _rooms.length,
                separatorBuilder: (_, __) => const SizedBox(width: 8),
                itemBuilder: (context, index) {
                  final room = _rooms[index];
                  final saved = _roomWaypoints.containsKey(room);
                  return Material(
                    color: const Color(0xff151b24),
                    borderRadius: BorderRadius.circular(8),
                    child: InkWell(
                      onTap: () => _editRoom(room),
                      borderRadius: BorderRadius.circular(8),
                      child: Container(
                        width: 154,
                        padding: const EdgeInsets.only(left: 12, right: 4),
                        decoration: BoxDecoration(
                          borderRadius: BorderRadius.circular(8),
                          border: Border.all(
                            color: saved
                                ? const Color(0xff22c55e)
                                : const Color(0xff384657),
                          ),
                        ),
                        child: Row(
                          children: [
                            Expanded(
                              child: Column(
                                mainAxisAlignment: MainAxisAlignment.center,
                                crossAxisAlignment: CrossAxisAlignment.start,
                                children: [
                                  Text(
                                    _roomLabel(room),
                                    maxLines: 1,
                                    overflow: TextOverflow.ellipsis,
                                    style: const TextStyle(
                                      fontWeight: FontWeight.w800,
                                    ),
                                  ),
                                  Text(
                                    saved
                                        ? (english ? 'Saved' : '已设置')
                                        : (english ? 'Tap to set' : '点按设置'),
                                    style: TextStyle(
                                      color: saved
                                          ? const Color(0xff22c55e)
                                          : Colors.white38,
                                      fontSize: 11,
                                    ),
                                  ),
                                ],
                              ),
                            ),
                            IconButton(
                              tooltip: english ? 'Navigate' : '导航到该房间',
                              onPressed: saved
                                  ? () => _navigateRoom(room)
                                  : null,
                              icon: const Icon(Icons.navigation_rounded),
                            ),
                          ],
                        ),
                      ),
                    ),
                  );
                },
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class OldSlamMapPage extends StatelessWidget {
  final bool isConnected;
  final String connectionText;

  const OldSlamMapPage({
    super.key,
    required this.isConnected,
    required this.connectionText,
  });

  @override
  Widget build(BuildContext context) {
    final statusColor = isConnected
        ? const Color(0xff22c55e)
        : Colors.redAccent;
    return Scaffold(
      backgroundColor: const Color(0xff0d1117),
      appBar: AppBar(
        title: const Text(
          'SLAM 地图',
          style: TextStyle(fontWeight: FontWeight.w800),
        ),
        backgroundColor: const Color(0xff10151d),
        elevation: 0,
      ),
      body: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          children: [
            Container(
              width: double.infinity,
              padding: const EdgeInsets.all(14),
              decoration: BoxDecoration(
                color: const Color(0xff151b24),
                borderRadius: BorderRadius.circular(8),
                border: Border.all(color: const Color(0xff263241)),
              ),
              child: Row(
                children: [
                  Icon(Icons.router_rounded, color: statusColor),
                  const SizedBox(width: 10),
                  Expanded(
                    child: Text(
                      'ROS2 通信状态：$connectionText',
                      style: const TextStyle(
                        color: Colors.white,
                        fontWeight: FontWeight.w700,
                      ),
                    ),
                  ),
                ],
              ),
            ),
            const SizedBox(height: 14),
            Expanded(
              child: Container(
                width: double.infinity,
                decoration: BoxDecoration(
                  color: Colors.black,
                  borderRadius: BorderRadius.circular(8),
                  border: Border.all(color: const Color(0xff263241)),
                ),
                child: CustomPaint(
                  painter: _SlamPreviewPainter(),
                  child: Center(
                    child: Column(
                      mainAxisSize: MainAxisSize.min,
                      children: [
                        Icon(
                          Icons.map_rounded,
                          size: 54,
                          color: const Color(
                            0xff38bdf8,
                          ).withValues(alpha: 0.72),
                        ),
                        const SizedBox(height: 10),
                        const Text(
                          '地图视图',
                          style: TextStyle(
                            color: Colors.white,
                            fontSize: 18,
                            fontWeight: FontWeight.w800,
                          ),
                        ),
                        const SizedBox(height: 6),
                        Text(
                          '用于展示 Cartographer / Nav2 建图结果',
                          style: TextStyle(
                            color: Colors.white.withValues(alpha: 0.48),
                            fontSize: 12,
                          ),
                        ),
                      ],
                    ),
                  ),
                ),
              ),
            ),
            const SizedBox(height: 14),
            Row(
              children: const [
                Expanded(
                  child: _MapStateCard(
                    icon: Icons.radar_rounded,
                    label: '建图',
                    value: '待接入',
                  ),
                ),
                SizedBox(width: 10),
                Expanded(
                  child: _MapStateCard(
                    icon: Icons.navigation_rounded,
                    label: '导航',
                    value: 'Nav2',
                  ),
                ),
                SizedBox(width: 10),
                Expanded(
                  child: _MapStateCard(
                    icon: Icons.my_location_rounded,
                    label: '定位',
                    value: 'AMCL',
                  ),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }
}

class _MapStateCard extends StatelessWidget {
  final IconData icon;
  final String label;
  final String value;

  const _MapStateCard({
    required this.icon,
    required this.label,
    required this.value,
  });

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.all(12),
      decoration: BoxDecoration(
        color: const Color(0xff151b24),
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: const Color(0xff263241)),
      ),
      child: Column(
        children: [
          Icon(icon, color: const Color(0xff38bdf8), size: 22),
          const SizedBox(height: 6),
          Text(
            label,
            style: TextStyle(
              color: Colors.white.withValues(alpha: 0.52),
              fontSize: 11,
            ),
          ),
          const SizedBox(height: 2),
          Text(
            value,
            maxLines: 1,
            overflow: TextOverflow.ellipsis,
            style: const TextStyle(
              color: Colors.white,
              fontSize: 13,
              fontWeight: FontWeight.w700,
            ),
          ),
        ],
      ),
    );
  }
}

class _SlamPreviewPainter extends CustomPainter {
  @override
  void paint(Canvas canvas, Size size) {
    final gridPaint = Paint()
      ..color = const Color(0xff1f2937)
      ..strokeWidth = 1;
    const step = 28.0;
    for (double x = 0; x < size.width; x += step) {
      canvas.drawLine(Offset(x, 0), Offset(x, size.height), gridPaint);
    }
    for (double y = 0; y < size.height; y += step) {
      canvas.drawLine(Offset(0, y), Offset(size.width, y), gridPaint);
    }

    final wallPaint = Paint()
      ..color = const Color(0xff38bdf8).withValues(alpha: 0.55)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 3;
    final path = Path()
      ..moveTo(size.width * 0.18, size.height * 0.24)
      ..lineTo(size.width * 0.74, size.height * 0.24)
      ..lineTo(size.width * 0.74, size.height * 0.68)
      ..lineTo(size.width * 0.36, size.height * 0.68)
      ..lineTo(size.width * 0.36, size.height * 0.44)
      ..lineTo(size.width * 0.18, size.height * 0.44)
      ..close();
    canvas.drawPath(path, wallPaint);

    final routePaint = Paint()
      ..color = const Color(0xff22c55e)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 3;
    final route = Path()
      ..moveTo(size.width * 0.28, size.height * 0.57)
      ..quadraticBezierTo(
        size.width * 0.48,
        size.height * 0.42,
        size.width * 0.64,
        size.height * 0.56,
      );
    canvas.drawPath(route, routePaint);

    final robotPaint = Paint()..color = Colors.amber;
    canvas.drawCircle(
      Offset(size.width * 0.28, size.height * 0.57),
      7,
      robotPaint,
    );
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => false;
}

class ArmControlPage extends StatefulWidget {
  final void Function(String payload) onArmCommand;

  const ArmControlPage({super.key, required this.onArmCommand});

  @override
  State<ArmControlPage> createState() => _ArmControlPageState();
}

class _ArmControlPageState extends State<ArmControlPage> {
  static const List<double> _homePose = [0.0, 1.0, -1.57, -1.57, 0.0, 0.0];
  final List<double> _angles = List<double>.from(_homePose);
  final List<double> _min = [-2.35, -1.57, -1.57, -1.57, -1.57, -1.15];
  final List<double> _max = [2.35, 1.57, 1.57, 1.57, 1.57, 0.75];
  Timer? _holdTimer;

  @override
  void dispose() {
    _holdTimer?.cancel();
    super.dispose();
  }

  void _publishArmCommand({required String type, int? joint, double? delta}) {
    widget.onArmCommand(
      jsonEncode({
        'type': type,
        'joint': joint,
        'delta': delta,
        'angles': _angles
            .map((v) => double.parse(v.toStringAsFixed(3)))
            .toList(),
        'timestamp': DateTime.now().toIso8601String(),
      }),
    );
  }

  void _nudgeJoint(int index, double delta) {
    setState(() {
      _angles[index] = (_angles[index] + delta).clamp(_min[index], _max[index]);
    });
    _publishArmCommand(type: 'joint_delta', joint: index + 1, delta: delta);
  }

  void _startHold(int index, double delta) {
    _nudgeJoint(index, delta);
    _holdTimer?.cancel();
    _holdTimer = Timer.periodic(
      const Duration(milliseconds: 150),
      (_) => _nudgeJoint(index, delta),
    );
  }

  void _stopHold() {
    _holdTimer?.cancel();
    _holdTimer = null;
  }

  void _resetArm() {
    setState(() {
      for (var i = 0; i < _angles.length; i++) {
        _angles[i] = _homePose[i];
      }
    });
    _publishArmCommand(type: 'reset');
  }

  void _gripper(bool close) {
    final index = 5;
    setState(() {
      _angles[index] = close ? _max[index] : _min[index];
    });
    widget.onArmCommand(close ? 'CLOSE' : 'RELEASE');
  }

  void _visionGrabBottle() {
    widget.onArmCommand('GRAB_BOTTLE');
  }

  void _stopVisionGrab() {
    widget.onArmCommand('STOP');
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: const Color(0xff0d1117),
      appBar: AppBar(
        title: const Text(
          '机械臂控制',
          style: TextStyle(fontWeight: FontWeight.w800),
        ),
        backgroundColor: const Color(0xff10151d),
        elevation: 0,
        actions: [
          IconButton(
            tooltip: '复位',
            onPressed: _resetArm,
            icon: const Icon(Icons.restart_alt_rounded),
          ),
        ],
      ),
      body: SafeArea(
        child: SingleChildScrollView(
          padding: const EdgeInsets.all(14),
          child: Column(
            children: [
              Container(
                width: double.infinity,
                padding: const EdgeInsets.all(14),
                decoration: BoxDecoration(
                  color: const Color(0xff151b24),
                  borderRadius: BorderRadius.circular(8),
                  border: Border.all(color: const Color(0xff263241)),
                ),
                child: Row(
                  children: [
                    Container(
                      width: 46,
                      height: 46,
                      decoration: BoxDecoration(
                        color: const Color(0xfff97316).withValues(alpha: 0.14),
                        borderRadius: BorderRadius.circular(8),
                      ),
                      child: const Icon(
                        Icons.precision_manufacturing_rounded,
                        color: Color(0xfff97316),
                      ),
                    ),
                    const SizedBox(width: 12),
                    const Expanded(
                      child: Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          Text(
                            'A150 六自由度机械臂',
                            style: TextStyle(
                              color: Colors.white,
                              fontWeight: FontWeight.w800,
                              fontSize: 16,
                            ),
                          ),
                          SizedBox(height: 4),
                          Text(
                            '关节角度控制 · MQTT',
                            style: TextStyle(
                              color: Colors.white54,
                              fontSize: 12,
                            ),
                          ),
                        ],
                      ),
                    ),
                  ],
                ),
              ),
              const SizedBox(height: 12),
              Row(
                children: [
                  Expanded(
                    flex: 2,
                    child: _buildArmCommandButton(
                      Icons.center_focus_strong_rounded,
                      '视觉抓取水瓶',
                      const Color(0xffa78bfa),
                      _visionGrabBottle,
                    ),
                  ),
                  const SizedBox(width: 10),
                  Expanded(
                    child: _buildArmCommandButton(
                      Icons.stop_circle_rounded,
                      '停止抓取',
                      const Color(0xffef4444),
                      _stopVisionGrab,
                    ),
                  ),
                ],
              ),
              const SizedBox(height: 12),
              Row(
                children: [
                  Expanded(
                    child: _buildArmCommandButton(
                      Icons.pan_tool_alt_rounded,
                      '松开水瓶',
                      const Color(0xff38bdf8),
                      () => _gripper(false),
                    ),
                  ),
                  const SizedBox(width: 10),
                  Expanded(
                    child: _buildArmCommandButton(
                      Icons.back_hand_rounded,
                      '夹取',
                      const Color(0xff22c55e),
                      () => _gripper(true),
                    ),
                  ),
                  const SizedBox(width: 10),
                  Expanded(
                    child: _buildArmCommandButton(
                      Icons.restart_alt_rounded,
                      '复位',
                      Colors.amber,
                      _resetArm,
                    ),
                  ),
                ],
              ),
              const SizedBox(height: 12),
              for (var i = 0; i < 6; i++) ...[
                _buildJointPad(i),
                const SizedBox(height: 10),
              ],
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildArmCommandButton(
    IconData icon,
    String label,
    Color color,
    VoidCallback onTap,
  ) {
    return Material(
      color: const Color(0xff151b24),
      borderRadius: BorderRadius.circular(8),
      child: InkWell(
        borderRadius: BorderRadius.circular(8),
        onTap: onTap,
        child: Container(
          height: 72,
          decoration: BoxDecoration(
            borderRadius: BorderRadius.circular(8),
            border: Border.all(color: const Color(0xff263241)),
          ),
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              Icon(icon, color: color),
              const SizedBox(height: 6),
              Text(
                label,
                style: const TextStyle(
                  color: Colors.white,
                  fontWeight: FontWeight.w700,
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildJointPad(int index) {
    final color = index == 5
        ? const Color(0xff22c55e)
        : const Color(0xfff97316);
    return Container(
      padding: const EdgeInsets.all(12),
      decoration: BoxDecoration(
        color: const Color(0xff151b24),
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: const Color(0xff263241)),
      ),
      child: Row(
        children: [
          SizedBox(
            width: 68,
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  'J${index + 1}',
                  style: TextStyle(
                    color: color,
                    fontSize: 18,
                    fontWeight: FontWeight.w900,
                  ),
                ),
                const SizedBox(height: 3),
                Text(
                  '${_angles[index].toStringAsFixed(2)} rad',
                  style: const TextStyle(color: Colors.white54, fontSize: 11),
                ),
              ],
            ),
          ),
          Expanded(
            child: SliderTheme(
              data: SliderTheme.of(context).copyWith(
                trackHeight: 3,
                thumbShape: const RoundSliderThumbShape(enabledThumbRadius: 7),
              ),
              child: Slider(
                value: _angles[index],
                min: _min[index],
                max: _max[index],
                activeColor: color,
                inactiveColor: Colors.white12,
                onChanged: (value) {
                  setState(() => _angles[index] = value);
                  _publishArmCommand(type: 'joint_set', joint: index + 1);
                },
              ),
            ),
          ),
          _buildNudgeButton(index, -0.05, Icons.remove_rounded, color),
          const SizedBox(width: 8),
          _buildNudgeButton(index, 0.05, Icons.add_rounded, color),
        ],
      ),
    );
  }

  Widget _buildNudgeButton(
    int index,
    double delta,
    IconData icon,
    Color color,
  ) {
    return GestureDetector(
      onTap: () => _nudgeJoint(index, delta),
      onLongPressStart: (_) => _startHold(index, delta),
      onLongPressEnd: (_) => _stopHold(),
      onLongPressCancel: _stopHold,
      child: Container(
        width: 38,
        height: 38,
        decoration: BoxDecoration(
          color: color.withValues(alpha: 0.12),
          borderRadius: BorderRadius.circular(8),
          border: Border.all(color: color.withValues(alpha: 0.44)),
        ),
        child: Icon(icon, color: color),
      ),
    );
  }
}

class _GrabOffsetDialog extends StatefulWidget {
  final List<double> initialValues;
  final bool isEnglish;

  const _GrabOffsetDialog({
    required this.initialValues,
    required this.isEnglish,
  });

  @override
  State<_GrabOffsetDialog> createState() => _GrabOffsetDialogState();
}

class _GrabOffsetDialogState extends State<_GrabOffsetDialog> {
  late final List<TextEditingController> _controllers;
  String? _errorText;

  @override
  void initState() {
    super.initState();
    _controllers = widget.initialValues
        .map((value) => TextEditingController(text: value.toStringAsFixed(3)))
        .toList();
  }

  @override
  void dispose() {
    for (final controller in _controllers) {
      controller.dispose();
    }
    super.dispose();
  }

  void _apply() {
    final parsed = _controllers
        .map((controller) => double.tryParse(controller.text.trim()))
        .toList();
    if (parsed.any((value) => value == null || value < -0.5 || value > 0.5)) {
      setState(() {
        _errorText = widget.isEnglish
            ? 'Enter numbers from -0.5 to 0.5.'
            : '请输入 -0.5 到 0.5 之间的数字。';
      });
      return;
    }
    Navigator.pop(context, parsed.map((value) => value!).toList());
  }

  @override
  Widget build(BuildContext context) {
    return AlertDialog(
      title: Text(widget.isEnglish ? 'Grab offsets' : '抓取偏移调参'),
      content: SingleChildScrollView(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Text(
              widget.isEnglish
                  ? 'Unit: meter. Changes apply to the next vision grab.'
                  : '单位：米。修改后从下一次视觉抓取开始生效。',
              style: const TextStyle(fontSize: 12),
            ),
            const SizedBox(height: 12),
            for (var i = 0; i < _controllers.length; i++) ...[
              TextField(
                controller: _controllers[i],
                keyboardType: const TextInputType.numberWithOptions(
                  signed: true,
                  decimal: true,
                ),
                decoration: InputDecoration(
                  labelText: '${String.fromCharCode(88 + i)} offset',
                  suffixText: 'm',
                  border: const OutlineInputBorder(),
                  isDense: true,
                ),
              ),
              if (i < _controllers.length - 1) const SizedBox(height: 10),
            ],
            if (_errorText != null) ...[
              const SizedBox(height: 10),
              Text(
                _errorText!,
                style: const TextStyle(color: Colors.redAccent, fontSize: 12),
              ),
            ],
          ],
        ),
      ),
      actions: [
        TextButton(
          onPressed: () => Navigator.pop(context),
          child: Text(widget.isEnglish ? 'Cancel' : '取消'),
        ),
        FilledButton(
          onPressed: _apply,
          child: Text(widget.isEnglish ? 'Apply' : '应用'),
        ),
      ],
    );
  }
}

class ArmControlPanel extends StatefulWidget {
  final void Function(String payload) onArmCommand;
  final bool dense;
  final bool isEnglish;

  const ArmControlPanel({
    super.key,
    required this.onArmCommand,
    this.dense = false,
    this.isEnglish = false,
  });

  @override
  State<ArmControlPanel> createState() => _ArmControlPanelState();
}

class _ArmControlPanelState extends State<ArmControlPanel> {
  static const List<double> _homePose = [0.0, 1.0, -1.57, -1.57, 0.0, 0.0];
  final List<double> _angles = List<double>.from(_homePose);
  final List<double> _min = [-2.35, -1.57, -1.57, -1.57, -1.57, -1.15];
  final List<double> _max = [2.35, 1.57, 1.57, 1.57, 1.57, 0.75];
  final List<double> _grabOffsets = [-0.07, -0.03, 0.05];
  Timer? _holdTimer;

  @override
  void dispose() {
    _holdTimer?.cancel();
    super.dispose();
  }

  void _publishArmCommand({required String type, int? joint, double? delta}) {
    widget.onArmCommand(
      jsonEncode({
        'type': type,
        'joint': joint,
        'delta': delta,
        'angles': _angles
            .map((value) => double.parse(value.toStringAsFixed(3)))
            .toList(),
        'timestamp': DateTime.now().toIso8601String(),
      }),
    );
  }

  void _nudgeJoint(int index, double delta) {
    setState(() {
      _angles[index] = (_angles[index] + delta).clamp(_min[index], _max[index]);
    });
    _publishArmCommand(type: 'joint_delta', joint: index + 1, delta: delta);
  }

  void _startHold(int index, double delta) {
    _nudgeJoint(index, delta);
    _holdTimer?.cancel();
    _holdTimer = Timer.periodic(
      const Duration(milliseconds: 150),
      (_) => _nudgeJoint(index, delta),
    );
  }

  void _stopHold() {
    _holdTimer?.cancel();
    _holdTimer = null;
  }

  void _resetArm() {
    setState(() {
      for (var i = 0; i < _angles.length; i++) {
        _angles[i] = _homePose[i];
      }
    });
    _publishArmCommand(type: 'reset');
  }

  void _visionGrabBottle() {
    widget.onArmCommand('GRAB_BOTTLE');
  }

  Future<void> _showOffsetTuning() async {
    final values = await showDialog<List<double>>(
      context: context,
      builder: (_) => _GrabOffsetDialog(
        initialValues: List<double>.from(_grabOffsets),
        isEnglish: widget.isEnglish,
      ),
    );
    if (values == null || !mounted) return;
    setState(() {
      for (var i = 0; i < _grabOffsets.length; i++) {
        _grabOffsets[i] = values[i];
      }
    });
    widget.onArmCommand(
      jsonEncode({
        'type': 'set_offsets',
        'x_offset': _grabOffsets[0],
        'y_offset': _grabOffsets[1],
        'z_offset': _grabOffsets[2],
        'timestamp': DateTime.now().toIso8601String(),
      }),
    );
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(
          widget.isEnglish ? 'Grab offsets sent to the robot' : '抓取偏移已下发到机器人',
        ),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Column(
      children: [
        Row(
          children: [
            Expanded(
              child: OutlinedButton.icon(
                onPressed: _resetArm,
                icon: const Icon(Icons.restart_alt_rounded, size: 17),
                label: Text(widget.isEnglish ? 'Care Position' : '回到看护姿态'),
              ),
            ),
            const SizedBox(width: 8),
            Expanded(
              child: OutlinedButton.icon(
                onPressed: _showOffsetTuning,
                icon: const Icon(Icons.tune_rounded, size: 17),
                label: Text(widget.isEnglish ? 'Offsets' : '抓取调参'),
              ),
            ),
            const SizedBox(width: 8),
            Expanded(
              child: FilledButton.icon(
                onPressed: _visionGrabBottle,
                icon: const Icon(Icons.center_focus_strong_rounded, size: 17),
                label: Text(widget.isEnglish ? 'Grab' : '抓取'),
                style: FilledButton.styleFrom(
                  backgroundColor: const Color(0xff7c3aed),
                  foregroundColor: Colors.white,
                ),
              ),
            ),
          ],
        ),
        const SizedBox(height: 8),
        Expanded(
          child: ListView.separated(
            padding: EdgeInsets.zero,
            itemBuilder: (context, index) => _jointRow(index),
            separatorBuilder: (_, __) => const SizedBox(height: 8),
            itemCount: 6,
          ),
        ),
      ],
    );
  }

  Widget _jointRow(int index) {
    final color = index == 5
        ? const Color(0xff22c55e)
        : const Color(0xfff97316);
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 8),
      decoration: BoxDecoration(
        color: const Color(0xff0f172a),
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: const Color(0xff263241)),
      ),
      child: Row(
        children: [
          SizedBox(
            width: 58,
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  'J${index + 1}',
                  style: TextStyle(
                    color: color,
                    fontWeight: FontWeight.w900,
                    fontSize: 16,
                  ),
                ),
                Text(
                  _angles[index].toStringAsFixed(2),
                  style: const TextStyle(color: Colors.white54, fontSize: 11),
                ),
              ],
            ),
          ),
          Expanded(
            child: SliderTheme(
              data: SliderTheme.of(context).copyWith(
                trackHeight: 3,
                thumbShape: const RoundSliderThumbShape(enabledThumbRadius: 7),
              ),
              child: Slider(
                value: _angles[index],
                min: _min[index],
                max: _max[index],
                activeColor: color,
                inactiveColor: Colors.white12,
                onChanged: (value) {
                  setState(() => _angles[index] = value);
                  _publishArmCommand(type: 'joint_set', joint: index + 1);
                },
              ),
            ),
          ),
          _nudgeButton(index, -0.05, Icons.remove_rounded, color),
          const SizedBox(width: 6),
          _nudgeButton(index, 0.05, Icons.add_rounded, color),
        ],
      ),
    );
  }

  Widget _nudgeButton(int index, double delta, IconData icon, Color color) {
    return GestureDetector(
      onTap: () => _nudgeJoint(index, delta),
      onLongPressStart: (_) => _startHold(index, delta),
      onLongPressEnd: (_) => _stopHold(),
      onLongPressCancel: _stopHold,
      child: Container(
        width: 34,
        height: 34,
        decoration: BoxDecoration(
          color: color.withValues(alpha: 0.12),
          borderRadius: BorderRadius.circular(8),
          border: Border.all(color: color.withValues(alpha: 0.44)),
        ),
        child: Icon(icon, color: color, size: 20),
      ),
    );
  }
}

class ParentVoiceClip {
  final Uint8List bytes;
  final int durationMs;

  const ParentVoiceClip({required this.bytes, required this.durationMs});
}

class ParentVoiceRecordDialog extends StatefulWidget {
  final bool isEnglish;

  const ParentVoiceRecordDialog({super.key, required this.isEnglish});

  @override
  State<ParentVoiceRecordDialog> createState() =>
      _ParentVoiceRecordDialogState();
}

class _ParentVoiceRecordDialogState extends State<ParentVoiceRecordDialog> {
  static const MethodChannel _channel = MethodChannel(
    'robot_control_app/native',
  );
  Timer? _timer;
  DateTime? _startedAt;
  bool _recording = false;
  bool _busy = false;
  int _elapsedMs = 0;
  String _error = '';

  String _t(String chinese, String english) =>
      widget.isEnglish ? english : chinese;

  Future<void> _start() async {
    if (_busy || _recording) return;
    setState(() {
      _busy = true;
      _error = '';
    });
    try {
      await _channel.invokeMethod('startVoiceRecording');
      if (!mounted) return;
      _startedAt = DateTime.now();
      setState(() {
        _recording = true;
        _busy = false;
        _elapsedMs = 0;
      });
      _timer = Timer.periodic(const Duration(milliseconds: 100), (_) {
        if (!mounted || !_recording || _startedAt == null) return;
        final elapsed = DateTime.now().difference(_startedAt!).inMilliseconds;
        setState(() => _elapsedMs = elapsed.clamp(0, 15000));
        if (elapsed >= 15000) unawaited(_finish());
      });
    } on PlatformException catch (e) {
      if (!mounted) return;
      setState(() {
        _busy = false;
        _error = e.message ?? _t('无法开始录音', 'Unable to start recording');
      });
    }
  }

  Future<void> _finish() async {
    if (_busy || !_recording) return;
    _timer?.cancel();
    setState(() {
      _busy = true;
      _recording = false;
    });
    try {
      final result = await _channel.invokeMapMethod<String, dynamic>(
        'stopVoiceRecording',
      );
      final rawAudio = result?['audio'];
      final audio = rawAudio is Uint8List
          ? rawAudio
          : Uint8List.fromList(List<int>.from(rawAudio as List? ?? const []));
      final durationMs = (result?['durationMs'] as num?)?.toInt() ?? _elapsedMs;
      if (audio.isEmpty) {
        throw PlatformException(
          code: 'AUDIO_EMPTY',
          message: _t('录音内容为空，请重试', 'Recording is empty; please retry'),
        );
      }
      if (!mounted) return;
      Navigator.of(
        context,
      ).pop(ParentVoiceClip(bytes: audio, durationMs: durationMs));
    } on PlatformException catch (e) {
      await _channel
          .invokeMethod('cancelVoiceRecording')
          .catchError((_) => null);
      if (!mounted) return;
      setState(() {
        _busy = false;
        _error = e.message ?? _t('录音失败，请重试', 'Recording failed; retry');
      });
    }
  }

  Future<void> _cancel() async {
    _timer?.cancel();
    if (_recording || _busy) {
      await _channel
          .invokeMethod('cancelVoiceRecording')
          .catchError((_) => null);
    }
    _recording = false;
    if (mounted) Navigator.of(context).pop();
  }

  @override
  void dispose() {
    _timer?.cancel();
    if (_recording) {
      unawaited(
        _channel.invokeMethod('cancelVoiceRecording').catchError((_) => null),
      );
    }
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final elapsed = (_elapsedMs / 1000).toStringAsFixed(1);
    return AlertDialog(
      backgroundColor: const Color(0xff151b24),
      title: Text(
        _t('录制家长原声', 'Record Parent Voice'),
        style: const TextStyle(
          color: Color(0xff22c55e),
          fontWeight: FontWeight.w800,
        ),
      ),
      content: SizedBox(
        width: 320,
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            InkResponse(
              onTap: _recording ? _finish : _start,
              radius: 48,
              child: Container(
                width: 88,
                height: 88,
                decoration: BoxDecoration(
                  shape: BoxShape.circle,
                  color: _recording
                      ? const Color(0xffdc2626)
                      : const Color(0xff22c55e),
                ),
                child: Icon(
                  _recording ? Icons.stop_rounded : Icons.mic_rounded,
                  size: 44,
                  color: Colors.white,
                ),
              ),
            ),
            const SizedBox(height: 14),
            Text(
              _recording
                  ? _t(
                      '正在录音  $elapsed / 15.0 秒',
                      'Recording  $elapsed / 15.0 s',
                    )
                  : _t('点击麦克风开始录音', 'Tap the microphone to record'),
              style: const TextStyle(
                color: Colors.white,
                fontSize: 15,
                fontWeight: FontWeight.w700,
              ),
            ),
            const SizedBox(height: 6),
            Text(
              _t('录音会由机器人原声播放', 'The robot will play the original voice'),
              textAlign: TextAlign.center,
              style: const TextStyle(color: Colors.white54, fontSize: 12),
            ),
            if (_error.isNotEmpty) ...[
              const SizedBox(height: 10),
              Text(
                _error,
                textAlign: TextAlign.center,
                style: const TextStyle(color: Colors.redAccent, fontSize: 12),
              ),
            ],
          ],
        ),
      ),
      actions: [
        TextButton(
          onPressed: _busy ? null : _cancel,
          child: Text(_t('取消', 'Cancel')),
        ),
        if (_recording)
          FilledButton.icon(
            onPressed: _busy ? null : _finish,
            icon: const Icon(Icons.send_rounded),
            label: Text(_t('停止并发送', 'Stop & Send')),
          ),
      ],
    );
  }
}

class CameraStreamPage extends StatefulWidget {
  final bool isEnglish;
  final void Function(double linear, double angular) onSpeedCommand;
  final ValueChanged<double> onLateralCommand;
  final void Function(String payload) onArmCommand;
  final void Function(Uint8List audio, int durationMs) onParentAudio;
  final ValueNotifier<List<Map<String, String>>> dialogueNotifier;
  final ValueNotifier<Map<String, dynamic>> armStatusNotifier;
  final String cameraBaseUrl;

  const CameraStreamPage({
    super.key,
    this.isEnglish = false,
    required this.onSpeedCommand,
    required this.onLateralCommand,
    required this.onArmCommand,
    required this.onParentAudio,
    required this.dialogueNotifier,
    required this.armStatusNotifier,
    required this.cameraBaseUrl,
  });

  @override
  State<CameraStreamPage> createState() => _CameraStreamPageState();
}

class _CameraStreamPageState extends State<CameraStreamPage> {
  bool get english => widget.isEnglish;
  String _t(String chinese, String englishText) =>
      english ? englishText : chinese;
  String get cameraBaseUrl => widget.cameraBaseUrl;
  String get snapshotUrl => '$cameraBaseUrl/snapshot.jpg';
  Timer? _timer;
  Timer? _lateralTimer;
  final HttpClient _httpClient = HttpClient()
    ..connectionTimeout = const Duration(seconds: 1);
  Uint8List? _frameBytes;
  bool _loadingFrame = false;
  bool _streaming = false;
  late String _statusText;
  late String _detailText;
  int _frameCount = 0;
  DateTime? _lastFrameAt;
  int _controlMode = 0;
  String _lastArmStatusSignature = '';

  @override
  void initState() {
    super.initState();
    _statusText = _t('摄像头未开始', 'Camera not started');
    _detailText = _t('等待摄像头地址', 'Waiting for camera address');
    _detailText = snapshotUrl;
    widget.armStatusNotifier.addListener(_handleArmStatus);
  }

  @override
  void dispose() {
    _timer?.cancel();
    _lateralTimer?.cancel();
    widget.onLateralCommand(0);
    widget.armStatusNotifier.removeListener(_handleArmStatus);
    _httpClient.close(force: true);
    super.dispose();
  }

  void _handleArmStatus() {
    if (!mounted) return;
    final status = widget.armStatusNotifier.value;
    final event = (status['event'] ?? '').toString();
    final message = (status['message'] ?? '').toString().trim();
    final timestamp = (status['timestamp'] ?? '').toString();
    final signature = '$event|$message|$timestamp';
    if (message.isEmpty || signature == _lastArmStatusSignature) return;
    _lastArmStatusSignature = signature;
    if (event != 'bottle_detected') return;
    HapticFeedback.mediumImpact();
    ScaffoldMessenger.of(context)
      ..hideCurrentSnackBar()
      ..showSnackBar(
        SnackBar(
          content: Text(_t('已识别到瓶子，正在确认位置', 'Bottle detected')),
          backgroundColor: const Color(0xff15803d),
          duration: const Duration(seconds: 3),
        ),
      );
  }

  Future<void> _startStream() async {
    if (_streaming) return;
    setState(() {
      _streaming = true;
      _frameBytes = null;
      _frameCount = 0;
      _statusText = _t('正在启动板端摄像头...', 'Starting robot camera...');
      _detailText = snapshotUrl;
    });
    try {
      await _sendCameraCommand('start');
      if (!mounted) return;
      setState(() {
        _statusText = _t('正在连接摄像头...', 'Connecting to camera...');
        _detailText = _t('板端采集已启动', 'Robot camera started');
      });
    } catch (e) {
      if (!mounted) return;
      setState(() {
        _streaming = false;
        _statusText = _t('启动摄像头失败', 'Failed to start camera');
        _detailText = '${e.runtimeType}: $e';
      });
      return;
    }
    _loadFrame();
    _timer = Timer.periodic(
      const Duration(milliseconds: 250),
      (_) => _loadFrame(),
    );
  }

  void _stopStream() {
    _timer?.cancel();
    _timer = null;
    unawaited(_sendCameraCommand('stop'));
    if (!mounted) return;
    setState(() {
      _streaming = false;
      _frameBytes = null;
      _statusText = _t('摄像头已停止', 'Camera stopped');
      _detailText = _t(
        '板端采集已关闭，点击播放按钮重新开始',
        'Robot camera stopped; tap Play to restart',
      );
    });
  }

  Future<void> _sendCameraCommand(String command) async {
    final request = await _httpClient.getUrl(
      Uri.parse('$cameraBaseUrl/control/$command'),
    );
    final response = await request.close().timeout(const Duration(seconds: 2));
    final body = await response.transform(utf8.decoder).join();
    if (response.statusCode != 200) {
      throw HttpException('HTTP ${response.statusCode}: $body');
    }
  }

  Future<void> _loadFrame() async {
    if (!_streaming && _frameBytes != null) return;
    if (_loadingFrame) return;
    _loadingFrame = true;
    try {
      final uri = Uri.parse(
        '$snapshotUrl?t=${DateTime.now().millisecondsSinceEpoch}',
      );
      final request = await _httpClient.getUrl(uri);
      request.headers.set(HttpHeaders.cacheControlHeader, 'no-cache');
      final response = await request.close().timeout(
        const Duration(seconds: 2),
      );
      if (response.statusCode == 200) {
        final bytes = await consolidateHttpClientResponseBytes(response);
        if (bytes.isNotEmpty && mounted) {
          setState(() {
            _frameBytes = bytes;
            _frameCount++;
            _lastFrameAt = DateTime.now();
            _statusText = _t('实时画面', 'Live video');
            _detailText = english
                ? 'HTTP 200  ${bytes.length} bytes  Frame $_frameCount'
                : 'HTTP 200  ${bytes.length} bytes  第 $_frameCount 帧';
          });
        }
      } else if (_frameBytes == null && mounted) {
        setState(() {
          _statusText = _t('摄像头服务返回异常', 'Camera service error');
          _detailText = english
              ? 'HTTP ${response.statusCode}, URL: $snapshotUrl'
              : 'HTTP ${response.statusCode}，地址：$snapshotUrl';
        });
      }
    } catch (e) {
      debugPrint('Camera snapshot load failed: $e');
      if (_frameBytes == null && mounted) {
        setState(() {
          _statusText = _t('连接摄像头失败', 'Camera connection failed');
          _detailText = '${e.runtimeType}: $e';
        });
      }
    } finally {
      _loadingFrame = false;
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: const Color(0xff0d1117),
      appBar: AppBar(
        title: Text(
          _t('儿童看护', 'Child Care'),
          style: const TextStyle(fontWeight: FontWeight.w800),
        ),
        backgroundColor: const Color(0xff10151d),
        elevation: 0,
        actions: [
          IconButton(
            tooltip: _t('刷新', 'Refresh'),
            onPressed: _streaming ? _loadFrame : null,
            icon: const Icon(Icons.refresh_rounded),
          ),
          IconButton(
            tooltip: _streaming
                ? _t('停止图传', 'Stop video')
                : _t('开始图传', 'Start video'),
            onPressed: _streaming
                ? _stopStream
                : () {
                    _startStream();
                  },
            icon: Icon(
              _streaming
                  ? Icons.pause_circle_filled_rounded
                  : Icons.play_circle_fill_rounded,
            ),
          ),
        ],
      ),
      body: SafeArea(
        child: Column(
          children: [
            Padding(
              padding: const EdgeInsets.fromLTRB(14, 12, 14, 0),
              child: AspectRatio(
                aspectRatio: 4 / 3,
                child: Container(
                  clipBehavior: Clip.antiAlias,
                  decoration: BoxDecoration(
                    color: Colors.black,
                    borderRadius: BorderRadius.circular(8),
                    border: Border.all(color: const Color(0xff263241)),
                  ),
                  child: Stack(
                    children: [
                      Positioned.fill(
                        child: InteractiveViewer(
                          minScale: 1,
                          maxScale: 4,
                          child: Center(
                            child: _frameBytes == null
                                ? Padding(
                                    padding: const EdgeInsets.all(22),
                                    child: Text(
                                      _streaming
                                          ? '$_statusText\n$_detailText'
                                          : '$_statusText\n${_t('点击右上角播放按钮开始', 'Tap Play in the top right to start')}',
                                      textAlign: TextAlign.center,
                                      style: const TextStyle(
                                        color: Colors.white70,
                                        height: 1.45,
                                      ),
                                    ),
                                  )
                                : Image.memory(
                                    _frameBytes!,
                                    gaplessPlayback: true,
                                    fit: BoxFit.contain,
                                  ),
                          ),
                        ),
                      ),
                      Positioned(
                        left: 10,
                        right: 10,
                        bottom: 10,
                        child: DecoratedBox(
                          decoration: BoxDecoration(
                            color: Colors.black.withValues(alpha: 0.52),
                            borderRadius: BorderRadius.circular(8),
                            border: Border.all(color: Colors.white12),
                          ),
                          child: Padding(
                            padding: const EdgeInsets.symmetric(
                              horizontal: 10,
                              vertical: 7,
                            ),
                            child: Text(
                              _frameBytes == null
                                  ? '${_t('状态', 'Status')}: $_statusText'
                                  : '${_t('状态', 'Status')}: $_statusText  ${_t('帧数', 'Frames')}: $_frameCount  ${_formatTime(_lastFrameAt)}',
                              maxLines: 1,
                              overflow: TextOverflow.ellipsis,
                              style: const TextStyle(
                                color: Colors.white,
                                fontSize: 12,
                              ),
                            ),
                          ),
                        ),
                      ),
                    ],
                  ),
                ),
              ),
            ),
            Expanded(
              child: Padding(
                padding: const EdgeInsets.all(14),
                child: Container(
                  width: double.infinity,
                  padding: const EdgeInsets.all(14),
                  decoration: BoxDecoration(
                    color: const Color(0xff151b24),
                    borderRadius: BorderRadius.circular(8),
                    border: Border.all(color: const Color(0xff263241)),
                  ),
                  child: Column(
                    children: [
                      Row(
                        children: [
                          const Icon(
                            Icons.tune_rounded,
                            color: Color(0xff38bdf8),
                          ),
                          const SizedBox(width: 8),
                          Expanded(
                            child: Text(
                              _t('看护控制', 'Care Controls'),
                              style: const TextStyle(
                                color: Colors.white,
                                fontWeight: FontWeight.w800,
                                fontSize: 15,
                              ),
                            ),
                          ),
                          Text(
                            _controlMode == 0
                                ? _t('松开摇杆自动停止', 'Release to stop')
                                : _t('机械臂辅助看护', 'Arm assistance'),
                            style: TextStyle(
                              color: Colors.white38,
                              fontSize: 11,
                            ),
                          ),
                        ],
                      ),
                      const SizedBox(height: 10),
                      SizedBox(
                        width: double.infinity,
                        child: SegmentedButton<int>(
                          showSelectedIcon: false,
                          segments: [
                            ButtonSegment(
                              value: 0,
                              icon: const Icon(Icons.sports_esports_rounded),
                              label: Text(_t('移动', 'Move')),
                            ),
                            ButtonSegment(
                              value: 1,
                              icon: const Icon(
                                Icons.precision_manufacturing_rounded,
                              ),
                              label: Text(_t('机械臂', 'Arm')),
                            ),
                          ],
                          selected: {_controlMode},
                          onSelectionChanged: (value) {
                            if (_controlMode == 0) {
                              widget.onSpeedCommand(0, 0);
                            }
                            setState(() => _controlMode = value.first);
                          },
                        ),
                      ),
                      const SizedBox(height: 10),
                      ValueListenableBuilder<Map<String, dynamic>>(
                        valueListenable: widget.armStatusNotifier,
                        builder: (context, status, _) {
                          final message = (status['message'] ?? '')
                              .toString()
                              .trim();
                          if (message.isEmpty ||
                              message.contains('机械臂视觉抓取节点已启动')) {
                            return const SizedBox.shrink();
                          }
                          final detected =
                              (status['event'] ?? '') == 'bottle_detected';
                          return Container(
                            width: double.infinity,
                            margin: const EdgeInsets.only(bottom: 8),
                            padding: const EdgeInsets.symmetric(
                              horizontal: 10,
                              vertical: 7,
                            ),
                            decoration: BoxDecoration(
                              color:
                                  (detected
                                          ? const Color(0xff15803d)
                                          : const Color(0xff1e3a5f))
                                      .withValues(alpha: 0.35),
                              borderRadius: BorderRadius.circular(8),
                              border: Border.all(
                                color: detected
                                    ? const Color(0xff22c55e)
                                    : const Color(0xff38bdf8),
                              ),
                            ),
                            child: Text(
                              message,
                              maxLines: 1,
                              overflow: TextOverflow.ellipsis,
                              style: const TextStyle(
                                color: Colors.white,
                                fontSize: 12,
                                fontWeight: FontWeight.w700,
                              ),
                            ),
                          );
                        },
                      ),
                      Expanded(
                        child: _controlMode == 0
                            ? _buildChassisControl()
                            : ArmControlPanel(
                                onArmCommand: widget.onArmCommand,
                                dense: true,
                                isEnglish: english,
                              ),
                      ),
                      const SizedBox(height: 10),
                      _buildDialoguePanel(),
                    ],
                  ),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Future<void> _showParentTalkDialog() async {
    final clip = await showDialog<ParentVoiceClip>(
      context: context,
      barrierDismissible: false,
      builder: (_) => ParentVoiceRecordDialog(isEnglish: english),
    );
    if (clip == null || clip.bytes.isEmpty) return;
    widget.onParentAudio(clip.bytes, clip.durationMs);
    if (!mounted) return;
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(
          english ? 'Original voice sent to the robot' : '家长原声已发送给机器人播放',
        ),
        duration: const Duration(seconds: 1),
        backgroundColor: const Color(0xff22c55e).withValues(alpha: 0.35),
      ),
    );
  }

  Widget _buildDialoguePanel() {
    return SizedBox(
      height: 118,
      child: Container(
        width: double.infinity,
        padding: const EdgeInsets.all(10),
        decoration: BoxDecoration(
          color: const Color(0xff0f172a),
          borderRadius: BorderRadius.circular(8),
          border: Border.all(color: const Color(0xff263241)),
        ),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                const Icon(
                  Icons.forum_rounded,
                  color: Color(0xff22c55e),
                  size: 17,
                ),
                const SizedBox(width: 6),
                Expanded(
                  child: Text(
                    _t('现场对话', 'Live Conversation'),
                    style: const TextStyle(
                      color: Colors.white,
                      fontSize: 13,
                      fontWeight: FontWeight.w800,
                    ),
                  ),
                ),
                TextButton.icon(
                  onPressed: _showParentTalkDialog,
                  icon: const Icon(Icons.record_voice_over_rounded, size: 16),
                  label: Text(_t('录家长原声', 'Record Voice')),
                  style: TextButton.styleFrom(
                    foregroundColor: const Color(0xff22c55e),
                    minimumSize: const Size(0, 32),
                    padding: const EdgeInsets.symmetric(horizontal: 8),
                    textStyle: const TextStyle(
                      fontSize: 12,
                      fontWeight: FontWeight.w800,
                    ),
                  ),
                ),
              ],
            ),
            const SizedBox(height: 6),
            Expanded(
              child: ValueListenableBuilder<List<Map<String, String>>>(
                valueListenable: widget.dialogueNotifier,
                builder: (context, messages, _) {
                  if (messages.isEmpty) {
                    return Center(
                      child: Text(
                        _t(
                          '等待机器人同步现场语音...',
                          'Waiting for live voice messages...',
                        ),
                        style: const TextStyle(
                          color: Colors.white38,
                          fontSize: 12,
                        ),
                      ),
                    );
                  }
                  final latest = messages.length > 3
                      ? messages.sublist(messages.length - 3)
                      : messages;
                  return ListView.separated(
                    reverse: true,
                    itemCount: latest.length,
                    separatorBuilder: (_, __) => const SizedBox(height: 5),
                    itemBuilder: (context, index) {
                      final msg = latest[latest.length - 1 - index];
                      final role = msg['role'] ?? 'user';
                      final isRobot = role == 'robot' || role == 'assistant';
                      final isParent = role == 'parent';
                      final name = isRobot
                          ? (english ? 'Robot' : '小微')
                          : isParent
                          ? (english ? 'Parent' : '家长')
                          : (english ? 'On-site user' : '现场用户');
                      final color = isRobot
                          ? const Color(0xff38bdf8)
                          : isParent
                          ? const Color(0xfff59e0b)
                          : const Color(0xff22c55e);
                      return Row(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          SizedBox(
                            width: 62,
                            child: Text(
                              msg['time'] ?? '--:--',
                              style: const TextStyle(
                                color: Colors.white38,
                                fontSize: 11,
                              ),
                            ),
                          ),
                          Expanded(
                            child: Text.rich(
                              TextSpan(
                                children: [
                                  TextSpan(
                                    text: '$name：',
                                    style: TextStyle(
                                      color: color,
                                      fontWeight: FontWeight.w800,
                                    ),
                                  ),
                                  TextSpan(text: msg['text'] ?? ''),
                                ],
                              ),
                              maxLines: 2,
                              overflow: TextOverflow.ellipsis,
                              style: const TextStyle(
                                color: Colors.white70,
                                fontSize: 12,
                                height: 1.25,
                              ),
                            ),
                          ),
                        ],
                      );
                    },
                  );
                },
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildChassisControl() {
    return Column(
      children: [
        Expanded(
          child: Center(
            child: Row(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                _lateralButton(
                  label: _t('左移', 'Left'),
                  icon: Icons.keyboard_double_arrow_left_rounded,
                  speed: 0.35,
                ),
                const SizedBox(width: 6),
                SizedBox(
                  width: 180,
                  height: 180,
                  child: VirtualJoystick(
                    onJoystickMoved: widget.onSpeedCommand,
                  ),
                ),
                const SizedBox(width: 6),
                _lateralButton(
                  label: _t('右移', 'Right'),
                  icon: Icons.keyboard_double_arrow_right_rounded,
                  speed: -0.35,
                ),
              ],
            ),
          ),
        ),
        Text(
          _t(
            '摇杆控制前后与转向，按住两侧按钮横移，松手自动停止',
            'Joystick drives and turns; hold side buttons to strafe',
          ),
          style: TextStyle(
            color: Colors.white.withValues(alpha: 0.48),
            fontSize: 12,
          ),
        ),
      ],
    );
  }

  Widget _lateralButton({
    required String label,
    required IconData icon,
    required double speed,
  }) {
    return Listener(
      behavior: HitTestBehavior.opaque,
      onPointerDown: (_) => _startLateral(speed),
      onPointerUp: (_) => _stopLateral(),
      onPointerCancel: (_) => _stopLateral(),
      child: Container(
        width: 48,
        height: 76,
        decoration: BoxDecoration(
          color: const Color(0xff38bdf8).withValues(alpha: 0.12),
          borderRadius: BorderRadius.circular(8),
          border: Border.all(
            color: const Color(0xff38bdf8).withValues(alpha: 0.45),
          ),
        ),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(icon, color: const Color(0xff38bdf8), size: 25),
            const SizedBox(height: 4),
            Text(
              label,
              style: const TextStyle(
                color: Colors.white70,
                fontSize: 11,
                fontWeight: FontWeight.w700,
              ),
            ),
          ],
        ),
      ),
    );
  }

  void _startLateral(double speed) {
    _lateralTimer?.cancel();
    widget.onLateralCommand(speed);
    _lateralTimer = Timer.periodic(
      const Duration(milliseconds: 150),
      (_) => widget.onLateralCommand(speed),
    );
  }

  void _stopLateral() {
    _lateralTimer?.cancel();
    _lateralTimer = null;
    widget.onLateralCommand(0);
  }

  String _formatTime(DateTime? value) {
    if (value == null) return '--:--:--';
    String two(int n) => n.toString().padLeft(2, '0');
    return '${two(value.hour)}:${two(value.minute)}:${two(value.second)}';
  }
}

// ==========================================
// 🕹️ 完全自主实现：带流光触控的精美虚拟摇杆组件
// ==========================================
class VirtualJoystick extends StatefulWidget {
  final Function(double linear, double angular) onJoystickMoved;
  const VirtualJoystick({super.key, required this.onJoystickMoved});

  @override
  State<VirtualJoystick> createState() => _VirtualJoystickState();
}

class _VirtualJoystickState extends State<VirtualJoystick> {
  Offset _dragPosition = Offset.zero;
  final double _joystickRadius = 90.0;
  final double _handleRadius = 30.0;

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onPanUpdate: (details) {
        Offset localPos =
            details.localPosition - Offset(_joystickRadius, _joystickRadius);
        double distance = localPos.distance;

        // 限制边界不能超出大圆圈
        if (distance > _joystickRadius - _handleRadius) {
          localPos = Offset.fromDirection(
            localPos.direction,
            _joystickRadius - _handleRadius,
          );
        }

        setState(() {
          _dragPosition = localPos;
        });

        // 🌟 核心物理映射：将拖动位移精准转换为标准的 ROS 速度线速度/角速度
        // 往上推为正速度，往下推为负速度；往左拉为正转速（向左偏航），往右拉为负转速
        double maxVelocity = 0.5; // 💥 最大线速度限速：0.5 m/s
        double maxAngular = 1.0; // 💥 最大转向速度限速：1.0 rad/s

        double linearSpeed =
            -(_dragPosition.dy / (_joystickRadius - _handleRadius)) *
            maxVelocity;
        double angularSpeed =
            -(_dragPosition.dx / (_joystickRadius - _handleRadius)) *
            maxAngular;

        // 回传触发发射！
        widget.onJoystickMoved(linearSpeed, angularSpeed);
      },
      onPanEnd: (details) {
        // 放手归位，速度立刻刹车清零，确保安全 🛑
        setState(() {
          _dragPosition = Offset.zero;
        });
        widget.onJoystickMoved(0.0, 0.0);
      },
      child: SizedBox(
        width: _joystickRadius * 2,
        height: _joystickRadius * 2,
        child: Stack(
          alignment: Alignment.center,
          children: [
            // 外部科技发光底盘大圆圈
            Container(
              width: _joystickRadius * 2,
              height: _joystickRadius * 2,
              decoration: BoxDecoration(
                shape: BoxShape.circle,
                color: const Color(0xff0d1117),
                border: Border.all(
                  color: const Color(0xff00f0ff).withValues(alpha: 0.4),
                  width: 3,
                ),
                boxShadow: [
                  BoxShadow(
                    color: const Color(0xff00f0ff).withValues(alpha: 0.1),
                    blurRadius: 20,
                    spreadRadius: 2,
                  ),
                ],
              ),
            ),
            // 十字辅助虚线
            Container(
              width: _joystickRadius * 2,
              height: 1,
              color: Colors.white12,
            ),
            Container(
              width: 1,
              height: _joystickRadius * 2,
              color: Colors.white12,
            ),

            // 核心内层可拖拽发光摇杆头 🟢
            Transform.translate(
              offset: _dragPosition,
              child: Container(
                width: _handleRadius * 2,
                height: _handleRadius * 2,
                decoration: BoxDecoration(
                  shape: BoxShape.circle,
                  gradient: const RadialGradient(
                    colors: [Color(0xff00ff66), Color(0xff00aa44)],
                  ),
                  boxShadow: [
                    BoxShadow(
                      color: const Color(0xff00ff66).withValues(alpha: 0.6),
                      blurRadius: 15,
                      spreadRadius: 1,
                    ),
                  ],
                ),
                child: const Icon(
                  Icons.drag_handle_rounded,
                  color: Colors.black87,
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
