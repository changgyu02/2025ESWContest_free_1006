import 'dart:convert';
import 'dart:math';
import 'package:flutter/material.dart';
import 'package:web_socket_channel/web_socket_channel.dart';
import 'package:project_bnw/User/user_info.dart';
import 'package:project_bnw/notification_log.dart';
import 'package:google_fonts/google_fonts.dart';
import 'package:project_bnw/final_main_home.dart';
import 'package:project_bnw/main.dart';

class MainHomePage extends StatefulWidget {
  const MainHomePage({super.key});

  @override
  State<MainHomePage> createState() => _MainHomePageState();
}

class _MainHomePageState extends State<MainHomePage>
    with SingleTickerProviderStateMixin, RouteAware {
  late WebSocketChannel channel;
  late AnimationController _controller;

  double battery1 = 0.0;
  double battery2 = 0.0;
  double alcohol = 0.0;
  double trash = 0.0;
  int mop = 0;
  List<Map<String, String>> notificationLogs = [];

  @override
  void initState() {
    super.initState();
    _controller = AnimationController(
      vsync: this,
      duration: const Duration(seconds: 2),
    );
    _controller.forward(); // 첫 진입에서 실행

    channel = WebSocketChannel.connect(
      Uri.parse('wss://9d061e0bf84b.ngrok-free.app/ws/status'),
    );

    channel.stream.listen((message) {
      final data = jsonDecode(message);
      setState(() {
        battery1 = ((data['battery_1'] ?? 0) / 100).clamp(0.0, 1.0);
        battery2 = ((data['battery_2'] ?? 0) / 100).clamp(0.0, 1.0);
        alcohol = ((data['alcohol'] ?? 0) / 100).clamp(0.0, 1.0);
        trash = ((data['trash'] ?? 0) / 100).clamp(0.0, 1.0);
        mop = (data['mop'] ?? 0).clamp(0, 10).toInt();
      });
    });
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    // 현재 route를 observer에 등록
    final route = ModalRoute.of(context);
    if (route is PageRoute) {
      routeObserver.subscribe(this, route);
    }
  }

  @override
  void dispose() {
    routeObserver.unsubscribe(this); // 구독 해제
    channel.sink.close();
    _controller.dispose();
    super.dispose();
  }

  @override
  void didPush() {
    _controller.forward(from: 0.0);
  }

  @override
  void didPopNext() {
    _controller.forward(from: 0.0);
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.white,
      body: SafeArea(
        child: Column(
          children: [
            // 상단 버튼 바
            Padding(
              padding: const EdgeInsets.symmetric(horizontal: 24.0, vertical: 16.0),
              child: Row(
                mainAxisAlignment: MainAxisAlignment.end,
                children: [
                  // 홈 버튼
                  IconButton(
                    icon: Image.asset(
                      'assets/images/home.png',
                      width: 24,
                      height: 24,
                      fit: BoxFit.contain,
                    ),
                    onPressed: () {
                      Navigator.push(
                        context,
                        MaterialPageRoute(builder: (context) => const FinalMainHome()),
                      );
                    },
                  ),
                  const SizedBox(width: 8),

                  // 알림 버튼
                  IconButton(
                    icon: Image.asset(
                      'assets/images/alarm.png',
                      width: 24,
                      height: 24,
                      fit: BoxFit.contain,
                    ),
                    onPressed: () {
                      Navigator.push(
                        context,
                        MaterialPageRoute(
                          builder: (context) => NotificationLogPage(logs: notificationLogs),
                        ),
                      );
                    },
                  ),
                  const SizedBox(width: 8),

                  // 유저 버튼
                  IconButton(
                    icon: Image.asset(
                      'assets/images/user.png',
                      width: 20,
                      height: 20,
                      fit: BoxFit.contain,
                    ),
                    onPressed: () {
                      Navigator.push(
                        context,
                        MaterialPageRoute(builder: (context) => const EditUserInfoPage()),
                      );
                    },
                  ),
                ],
              ),
            ),
            Expanded(
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  AnimatedBuilder(
                    animation: _controller,
                    builder: (context, child) {
                      final value = Curves.easeOutCubic.transform(_controller.value);
                      return Column(
                        children: [
                          CustomPaint(
                            painter: QuadArcPainter(
                              battery1: battery1 * value,
                              battery2: battery2 * value,
                              alcohol: alcohol * value,
                              trash: trash * value,
                            ),
                            size: const Size(240, 240),
                          ),
                          const SizedBox(height: 24),
                          _buildStatusRow('배터리1', battery1, const Color(0xFF70C1B3)),
                          _buildStatusRow('배터리2', battery2, const Color(0xFF379C8F)),
                          _buildStatusRow('알코올', alcohol, const Color(0xFF4DBCE9)),
                          _buildStatusRow('쓰레기통', trash, const Color(0xFFFFDD7F)),
                        ],
                      );
                    },
                  ),
                  const SizedBox(height: 32),
                  Text(
                    '여분 걸레 수: $mop개',
                    style: GoogleFonts.notoSansKr(
                      fontSize: 18,
                      color: Colors.black87,
                    ),
                  ),
                  const SizedBox(height: 40),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildStatusRow(String label, double value, Color color) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 4.0),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Container(
            width: 12,
            height: 12,
            decoration: BoxDecoration(
              color: color,
              shape: BoxShape.circle,
            ),
          ),
          const SizedBox(width: 6),
          Text(
            '$label: ${(value * 100).round()}%',
            style: GoogleFonts.notoSansKr(
              fontSize: 15,
              color: Colors.black87,
            ),
          ),
        ],
      ),
    );
  }
}

class QuadArcPainter extends CustomPainter {
  final double battery1;
  final double battery2;
  final double alcohol;
  final double trash;

  QuadArcPainter({
    required this.battery1,
    required this.battery2,
    required this.alcohol,
    required this.trash,
  });

  @override
  void paint(Canvas canvas, Size size) {
    final strokeWidth = 10.0;
    final center = Offset(size.width / 2, size.height / 2);
    final baseRadius = size.width / 2 - strokeWidth;

    final paint = Paint()
      ..style = PaintingStyle.stroke
      ..strokeWidth = strokeWidth
      ..strokeCap = StrokeCap.round;

    paint.color = const Color(0xFF70C1B3); // 배터리1: 민트
    canvas.drawArc(
      Rect.fromCircle(center: center, radius: baseRadius),
      -pi / 2,
      2 * pi * battery1,
      false,
      paint,
    );

    paint.color = const Color(0xFF379C8F); // 배터리2: 진한 민트
    canvas.drawArc(
      Rect.fromCircle(center: center, radius: baseRadius - 18),
      -pi / 2,
      2 * pi * battery2,
      false,
      paint,
    );

    paint.color = const Color(0xFF4DBCE9); // 알코올
    canvas.drawArc(
      Rect.fromCircle(center: center, radius: baseRadius - 36),
      -pi / 2,
      2 * pi * alcohol,
      false,
      paint,
    );

    paint.color = const Color(0xFFFFDD7F); // 쓰레기
    canvas.drawArc(
      Rect.fromCircle(center: center, radius: baseRadius - 54),
      -pi / 2,
      2 * pi * trash,
      false,
      paint,
    );
  }

  @override
  bool shouldRepaint(CustomPainter oldDelegate) => true;
}