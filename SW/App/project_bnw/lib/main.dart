import 'package:flutter/material.dart';
import 'package:google_fonts/google_fonts.dart';
import 'package:permission_handler/permission_handler.dart';
import 'package:flutter_local_notifications/flutter_local_notifications.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:project_bnw/User/login.dart';
import 'package:project_bnw/main_home.dart';

class NotificationService {
  static final FlutterLocalNotificationsPlugin _notifications = FlutterLocalNotificationsPlugin();

  static const String _channelId = 'status_channel';
  static const String _groupKey = 'robot_status_group';
  static int _notificationId = 1; // 고유 ID 증가용
  static final List<String> _titles = []; // 요약용 타이틀 목록

  Future<void> init() async {
    const androidSettings = AndroidInitializationSettings('@mipmap/ic_launcher');
    const initializationSettings = InitializationSettings(android: androidSettings);
    await _notifications.initialize(initializationSettings);
  }

  Future<void> showGroupedNotification({
    required String title,
    required String body,
  }) async {
    // 1. 개별 알림 전송
    await _notifications.show(
      _notificationId++,
      title,
      body,
      const NotificationDetails(
        android: AndroidNotificationDetails(
          _channelId,
          '청소로봇 알림',
          channelDescription: '상태 모니터링 알림',
          importance: Importance.max,
          priority: Priority.high,
          groupKey: _groupKey,
        ),
      ),
    );

    _titles.add('$title: $body');

    // 2. 그룹 요약 알림 재전송 (항상 최신으로 유지)
    await _notifications.show(
      0,
      'B&W 로봇 상태 알림',
      '${_titles.length}개의 알림이 도착했습니다.',
      NotificationDetails(
        android: AndroidNotificationDetails(
          _channelId,
          '청소로봇 알림',
          channelDescription: '상태 모니터링 알림',
          groupKey: _groupKey,
          setAsGroupSummary: true,
          styleInformation: InboxStyleInformation(
            _titles, // 개별 알림 타이틀 리스트
            summaryText: '${_titles.length}개 항목',
          ),
        ),
      ),
    );
  }
}

final notificationService = NotificationService();
final RouteObserver<PageRoute> routeObserver = RouteObserver<PageRoute>();

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();

  final prefs = await SharedPreferences.getInstance();
  final keepLogin = prefs.getBool('keepLogin') ?? false;

  final status = await Permission.notification.request();
  if (status != PermissionStatus.granted) {
    debugPrint("알림 권한이 거부되었습니다.");
  }

  await notificationService.init();

  runApp(MyApp(keepLogin: keepLogin));
}

class MyApp extends StatelessWidget {
  final bool keepLogin;
  const MyApp({super.key, required this.keepLogin});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'B&W 테이블 청소 로봇',
      debugShowCheckedModeBanner: false,
      navigatorObservers: [routeObserver], // 👈 여기서 등록
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(
          seedColor: const Color(0xFF70C1B3),
        ),
        useMaterial3: true,
        textTheme: GoogleFonts.notoSansKrTextTheme(),
        scaffoldBackgroundColor: const Color(0xFFF9F9F9),
      ),
      home: keepLogin ? const MainHomePage() : const LoginPage(),
    );
  }
}