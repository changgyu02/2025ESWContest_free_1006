import 'dart:convert';
import 'dart:math';
import 'package:flutter/material.dart';
import 'package:google_fonts/google_fonts.dart';
import 'package:web_socket_channel/web_socket_channel.dart';
import 'package:http/http.dart' as http;
import 'dart:ui' as ui;
import 'package:flutter/services.dart';
import 'package:project_bnw/User/user_info.dart'; // 경로에 맞게 조정
import 'package:project_bnw/notification_log.dart';
import 'main.dart';

class FinalMainHome extends StatefulWidget {
  const FinalMainHome({super.key});

  @override
  State<FinalMainHome> createState() => _FinalMainHomeState();
}

class _FinalMainHomeState extends State<FinalMainHome>
    with SingleTickerProviderStateMixin {
  late WebSocketChannel channel;
  double batteryLevel1 = 0.0;
  double batteryLevel2 = 0.0;
  double alcoholLevel = 0.0;
  double trashLevel = 0.0;
  double mopLevel = 0.0;
  String batteryStatus1 = "대기 중입니다";
  String batteryStatus2 = "대기 중입니다";
  String alcoholStatus = "대기 중입니다";
  String trashStatus = "대기 중입니다";
  String mopStatus = "대기 중입니다";
  bool batteryAlarm1 = false;
  bool batteryAlarm2 = false;
  bool alcoholAlarm = false;
  bool trashAlarm = false;
  bool mopAlarm = false;
  List<Map<String, String>> notificationLogs = [];
  late AnimationController _animationController;

  void _showAlert(String title, String body, DateTime time) async {
    await notificationService.showGroupedNotification(title: title, body: body);

    setState(() {
      notificationLogs.add({
        'time': time.toString(),
        'title': title,
        'body': body,
      });
    });
  }

  bool _isListening = false; // 클래스 필드로 추가

  @override
  void initState() {
    super.initState();

    channel = WebSocketChannel.connect(
      Uri.parse('wss://9d061e0bf84b.ngrok-free.app/ws/status'),
    );

    if (!_isListening) {
      _isListening = true;

      channel.stream.listen(
            (message) {
          final data = jsonDecode(message);
          final now = DateTime.now();
          setState(() {
            batteryLevel1 = ((data['battery_1'] ?? 0) / 100).clamp(0.0, 1.0);
            batteryLevel2 = ((data['battery_2'] ?? 0) / 100).clamp(0.0, 1.0);
            alcoholLevel = ((data['alcohol'] ?? 0) / 100).clamp(0.0, 1.0);
            trashLevel = ((data['trash'] ?? 0) / 100).clamp(0.0, 1.0);
            mopLevel = ((data['mop'] ?? 0).toDouble()).clamp(0.0, 10.0);

            batteryStatus1 = data['status_battery_1'] ?? "정보 없음";
            batteryStatus2 = data['status_battery_2'] ?? "정보 없음";
            alcoholStatus = data['status_alcohol'] ?? "정보 없음";
            trashStatus = data['status_trash'] ?? "정보 없음";
            mopStatus = data['status_mop'] ?? "정보 없음";

            final batteryPercent1 = (data['battery_1'] ?? 0);
            final batteryPercent2 = (data['battery_2'] ?? 0);
            final alcoholPercent = (data['alcohol'] ?? 0);
            final trashPercent = (data['trash'] ?? 0);
            final mopCount = (data['mop'] ?? 0).round();

            if (batteryPercent1 <= 20 && !batteryAlarm1) {
              batteryAlarm1 = true;
              _showAlert("배터리1 전원 부족", "배터리1 20% 남음", now);
            } else if (batteryPercent1 > 20) {
              batteryAlarm1 = false;
            }

            if (batteryPercent2 <= 20 && !batteryAlarm2) {
              batteryAlarm2 = true;
              _showAlert("배터리2 전원 부족", "배터리2 20% 남음", now);
            } else if (batteryPercent2 > 20) {
              batteryAlarm2 = false;
            }

            if (alcoholPercent <= 20 && !alcoholAlarm) {
              alcoholAlarm = true;
              _showAlert("알코올 부족", "알코올 20% 남음", now);
            } else if (alcoholPercent > 20) {
              alcoholAlarm = false;
            }

            if (trashPercent >= 80 && !trashAlarm) {
              trashAlarm = true;
              _showAlert("쓰레기 적재량 초과", "쓰레기통 80% 이상", now);
            } else if (trashPercent < 80) {
              trashAlarm = false;
            }

            if (mopCount <= 2 && !mopAlarm) {
              mopAlarm = true;
              _showAlert("걸레 부족", "걸레 교체 필요", now);
            } else if (mopCount > 2) {
              mopAlarm = false;
            }
          });
        },
        onError: (error) {
          debugPrint("WebSocket 오류 발생: $error");
          _isListening = false;
          // 필요 시: 사용자에게 알림 표시 or 재연결 시도
        },
        onDone: () {
          debugPrint("WebSocket 연결 종료됨");
          _isListening = false;
          // 필요 시: 재연결 시도 로직
        },
        cancelOnError: true,
      );
    }

    _animationController = AnimationController(
      vsync: this,
      duration: const Duration(seconds: 3),
    )
      ..repeat();
  }

  @override
  void dispose() {
    channel.sink.close();
    _animationController.dispose();
    super.dispose();
  }

  Widget _buildDualBatteryPage() {
    return Center(
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Row(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              BatteryCardWidget(
                label: '배터리 1',
                level: batteryLevel1,
                imagePath: 'assets/images/robot.png',
              ),
              BatteryCardWidget(
                label: '배터리 2',
                level: batteryLevel2,
                imagePath: 'assets/images/robotic_arm.png',
              ),
            ],
          ),
        ],
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: const Color(0xFFFFFFFF),
      body: SafeArea(
        child: Column(
          children: [
            Padding(
              padding: const EdgeInsets.symmetric(horizontal: 24.0, vertical: 16.0),
              child: Row(
                mainAxisAlignment: MainAxisAlignment.end,
                children: [
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
              child: PageView(
                children: [
                  _buildDualBatteryPage(),
                  _buildStatusPage(
                    title: "알코올 잔량",
                    level: alcoholLevel,
                    status: alcoholStatus,
                    widgetBuilder: () => CuteAlcoholWidget(
                      level: alcoholLevel,
                      animation: _animationController,
                    ),
                  ),
                  _buildStatusPage(
                    title: "쓰레기통 적재량",
                    level: trashLevel,
                    status: trashStatus,
                    widgetBuilder: () => CuteTrashWidget(
                      level: trashLevel,
                      animation: _animationController,
                    ),
                  ),
                  _buildStatusPage(
                    title: "${mopLevel.toInt()}개",
                    level: mopLevel.toDouble(),
                    status: mopStatus,
                    widgetBuilder: () => CuteMopWidget(
                      level: mopLevel.toDouble(),
                      animation: _animationController,
                    ),
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }

  Future<void> _fillMopToEight() async {
    try {
      // ngrok 서버 주소 직접 사용
      final uri = Uri.parse('https://9d061e0bf84b.ngrok-free.app/mop/fill');

      final resp = await http.post(
        uri,
        headers: {'Content-Type': 'application/json'},
        body: jsonEncode({'mop': 10}),
      );

      if (resp.statusCode == 200) {
        // UI 반영 (서버가 주기적으로 WebSocket으로 보내주니 생략해도 곧 동기화됨)
        setState(() {
          mopLevel = 10.0;
          mopStatus = '정상';
        });
      } else {
        debugPrint('채움 실패: ${resp.statusCode} ${resp.body}');
      }
    } catch (e) {
      debugPrint('채움 요청 에러: $e');
    }
  }

  Future<void> _fillAlcoholTo100() async {
    try {
      final uri = Uri.parse('https://9d061e0bf84b.ngrok-free.app/alcohol/fill');
      final resp = await http.post(
        uri,
        headers: {'Content-Type': 'application/json'},
        body: jsonEncode({'alcohol': 100}),
      );

      if (resp.statusCode == 200) {
        // 낙관적 UI 반영(선택) – 서버가 WebSocket으로 다시 푸시하니 생략해도 곧 동기화됨
        setState(() {
          alcoholLevel = 1.0;    // 100%
          alcoholStatus = '정상';
        });
      } else {
        debugPrint('알코올 채움 실패: ${resp.statusCode} ${resp.body}');
      }
    } catch (e) {
      debugPrint('알코올 채움 요청 에러: $e');
    }
  }

  Widget _buildStatusPage({
    required String title,
    required double level,
    required String status,
    required Widget Function() widgetBuilder,
  }) {
    const mainColor = Color(0xFF70C1B3);
    final isAlcohol = title.contains("알코올");
    final isMop = title.contains("개"); // 걸레 화면 판단

    return Center(
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          // 1) 그림
          widgetBuilder(),
          const SizedBox(height: 20),

          // 2) 개수 또는 %
          Text(
            isMop ? title : "${(level * 100).toInt()}%",
            style: GoogleFonts.notoSansKr(
              fontSize: 32,
              fontWeight: FontWeight.bold,
              color: mainColor,
            ),
          ),
          const SizedBox(height: 12),

          // 3) 상태
          Text(
            status,
            style: GoogleFonts.notoSansKr(
              fontSize: 16,
              color: Colors.black87,
            ),
          ),
          // 4) 채움 버튼(걸레 화면에서만, 맨 아래)
          if (isMop) ...[
            const SizedBox(height: 10),
            SizedBox(
              width: 120,
              height: 40,
              child: OutlinedButton(
                onPressed: _fillMopToEight, // ← 기존 mop용 함수(이미 적용했다면 그대로)
                style: OutlinedButton.styleFrom(
                  foregroundColor: Colors.black,
                  backgroundColor: Colors.white,
                  side: const BorderSide(color: Color(0xFF70C1B3), width: 2),
                  shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
                ),
                child: const Text('채움'),
              ),
            ),
          ],

          if (isAlcohol) ...[
            const SizedBox(height: 10),
            SizedBox(
              width: 120,
              height: 40,
              child: OutlinedButton(
                onPressed: _fillAlcoholTo100, // ← 여기만 알코올 함수로 연결
                style: OutlinedButton.styleFrom(
                  foregroundColor: Colors.black,
                  backgroundColor: Colors.white,
                  side: const BorderSide(color: Color(0xFF70C1B3), width: 2),
                  shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
                ),
                child: const Text('채움'),
              ),
            ),
          ],
        ],
      ),
    );
  }
}

class BatteryCardWidget extends StatefulWidget {
  final String label;
  final double level;
  final String imagePath; // ✅ 추가

  const BatteryCardWidget({
    super.key,
    required this.label,
    required this.level,
    required this.imagePath, // ✅ 추가
  });

  @override
  State<BatteryCardWidget> createState() => _BatteryCardWidgetState();
}

class _BatteryCardWidgetState extends State<BatteryCardWidget>
    with SingleTickerProviderStateMixin {
  late final AnimationController _faceController;

  @override
  void initState() {
    super.initState();
    _faceController = AnimationController(
      vsync: this,
      duration: const Duration(seconds: 2),
    )..repeat(); // 얼굴(눈물) 애니메이션 반복
  }

  @override
  void dispose() {
    _faceController.dispose();
    super.dispose();
  }

  String _getStatusText(double level) {
    if (level >= 0.3) return '정상';
    return '충전 필요';
  }

  @override
  Widget build(BuildContext context) {
    return TweenAnimationBuilder<double>(
      tween: Tween<double>(begin: 0, end: widget.level),
      duration: const Duration(milliseconds: 1200),
      curve: Curves.easeOutCubic,
      builder: (context, animatedValue, _) {
        final percentage = (animatedValue * 100).toInt();

        return Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Stack(
              clipBehavior: Clip.none,
              children: [
                Container(
                  width: 160,
                  height: 270,
                  margin: const EdgeInsets.symmetric(horizontal: 10),
                  decoration: BoxDecoration(
                    color: Colors.grey[300],
                    borderRadius: BorderRadius.circular(24),
                  ),
                  child: Stack(
                    children: [
                      Align(
                        alignment: Alignment.bottomCenter,
                        child: FractionallySizedBox(
                          heightFactor: animatedValue.clamp(0.0, 1.0),
                          alignment: Alignment.bottomCenter,
                          child: Container(
                            decoration: const BoxDecoration(
                              color: Color(0x9970C1B3),
                              borderRadius: BorderRadius.vertical(
                                bottom: Radius.circular(24),
                                top: Radius.circular(10),
                              ),
                            ),
                          ),
                        ),
                      ),
                      Align(
                        alignment: Alignment.center,
                        child: CustomPaint(
                          size: const Size(150, 200), // 배터리 크기와 맞추기
                          painter: BatteryFacePainter(
                            level: animatedValue,
                            animation: _faceController,
                          ),
                        ),
                      ),
                      // 흰 원
                      // ✅ 이미지 포함된 흰 원
                      Align(
                        alignment: Alignment.bottomCenter,
                        child: Container(
                          width: 48,
                          height: 48,
                          margin: const EdgeInsets.only(bottom: 16),
                          decoration: BoxDecoration(
                            color: Colors.white.withAlpha(128),
                            shape: BoxShape.circle,
                          ),
                          child: ClipOval(
                            child: Padding(
                              padding: const EdgeInsets.all(6), // 이미지 너무 꽉 차지 않게 여백
                              child: Image.asset(
                                widget.imagePath,
                                fit: BoxFit.contain,
                              ),
                            ),
                          ),
                        ),
                      ),
                    ],
                  ),
                ),
                // 상단 배터리 팁
                Positioned(
                  top: -12,
                  left: 0,
                  right: 0,
                  child: Align(
                    alignment: Alignment.center,
                    child: Container(
                      width: 34,
                      height: 10,
                      decoration: BoxDecoration(
                        color: Colors.grey.shade600,
                        borderRadius: BorderRadius.circular(4),
                      ),
                    ),
                  ),
                ),
              ],
            ),
            const SizedBox(height: 14),
            Text(
              '$percentage%',
              style: GoogleFonts.notoSansKr(
                fontSize: 24,
                fontWeight: FontWeight.w500,
                color: Colors.black87,
              ),
            ),
            const SizedBox(height: 6),
            Text(
              widget.label,
              style: GoogleFonts.notoSansKr(
                fontSize: 16,
                fontWeight: FontWeight.w400,
                color: Colors.black87,
              ),
            ),
            Text(
              _getStatusText(animatedValue),
              style: GoogleFonts.notoSansKr(
                fontSize: 14,
                fontWeight: FontWeight.w400,
                color: Colors.black54,
              ),
            ),
          ],
        );
      },
    );
  }
}

class BatteryFacePainter extends CustomPainter {
  final double level;
  final Animation<double> animation;
  BatteryFacePainter({required this.level, required this.animation}) : super(repaint: animation);

  @override
  void paint(Canvas canvas, Size size) {
    final eyePaint = Paint()..color = Colors.black;
    final cheekPaint = Paint()..color = const Color(0xFFFFC1CC);
    final mouthPaint = Paint()
      ..color = Colors.black
      ..style = PaintingStyle.stroke
      ..strokeWidth = 4.0;
    final tearPaint = Paint()..color = Colors.blueAccent;
    final centerX = size.width * 0.5;
    final eyeY = size.height * 0.35;
    final cheekY = size.height * 0.37;
    final mouthY = size.height * 0.44;

    // 볼터치
    canvas.drawOval(
      Rect.fromCenter(center: Offset(size.width * 0.2, cheekY), width: 16, height: 10),
      cheekPaint,
    );
    canvas.drawOval(
      Rect.fromCenter(center: Offset(size.width * 0.8, cheekY), width: 16, height: 10),
      cheekPaint,
    );

    // 눈
    final leftEyeX = size.width * 0.25;
    final rightEyeX = size.width * 0.75;

    // 타원의 가로 세로 길이 설정
    const eyeWidth = 9.5;
    const eyeHeight = 12.5;

    // 왼쪽 눈
    canvas.drawOval(
      Rect.fromCenter(
        center: Offset(leftEyeX, eyeY),
        width: eyeWidth,
        height: eyeHeight,
      ),
      eyePaint,
    );

    // 오른쪽 눈
    canvas.drawOval(
      Rect.fromCenter(
        center: Offset(rightEyeX, eyeY),
        width: eyeWidth,
        height: eyeHeight,
      ),
      eyePaint,
    );

    // 입
    final mouthPath = Path();
    if (level >= 0.7) {
      // 😊 웃는 입
      mouthPath.moveTo(centerX - 10, mouthY);
      mouthPath.quadraticBezierTo(centerX, mouthY + 6, centerX + 10, mouthY);
    } else if (level <= 0.3) {
      // 😢 우는 입
      mouthPath.moveTo(centerX - 12, mouthY + 6);
      mouthPath.quadraticBezierTo(centerX, mouthY - 4, centerX + 12, mouthY + 6);
    } else {
      // 😐 무표정
      mouthPath.moveTo(centerX - 10, mouthY);
      mouthPath.lineTo(centerX + 10, mouthY);
    }
    canvas.drawPath(mouthPath, mouthPaint);

    // 👇 눈물 애니메이션 (level <= 0.3일 때만 표시)
    if (level <= 0.3) {
      final tearDrop = (animation.value % 1.0) * 20;

      final tearLeftX = leftEyeX + 1; // 눈보다 살짝 오른쪽
      final tearLeftY = eyeY + 5 + tearDrop;
      if (tearDrop < 10) {
        canvas.drawOval(
          Rect.fromCenter(center: Offset(tearLeftX, tearLeftY), width: 6, height: 10),
          tearPaint,
        );
      }

      final tearRightProgress = ((animation.value + 0.5) % 0.75);
      final tearRightDrop = tearRightProgress * 20;
      final tearRightX = rightEyeX - 1;
      final tearRightY = eyeY + 5 + tearRightDrop;
      if (tearRightDrop < 10) {
        canvas.drawOval(
          Rect.fromCenter(center: Offset(tearRightX, tearRightY), width: 6, height: 10),
          tearPaint,
        );
      }
    }
  }

  @override
  bool shouldRepaint(CustomPainter oldDelegate) => true;
}

class CuteAlcoholWidget extends StatelessWidget {
  final double level;
  final Animation<double> animation;

  const CuteAlcoholWidget({
    super.key,
    required this.level,
    required this.animation,
  });

  @override
  Widget build(BuildContext context) {
    return CustomPaint(
      size: const Size(240, 280),
      painter: _CuteAlcoholPainter(level, animation),
    );
  }
}

class _CuteAlcoholPainter extends CustomPainter {
  final double level;
  final Animation<double> animation;

  _CuteAlcoholPainter(this.level, this.animation) : super(repaint: animation);

  @override
  void paint(Canvas canvas, Size size) {
    final width = size.width;
    final height = size.height;
    final scaleX = width / 250.0;
    final scaleY = height / 250.0;
    final backgroundPaint = Paint()..color = Colors.grey[300]!;
    final wavePaint1 = Paint()..color = const Color(0x9970C1B3);
    final wavePaint2 = Paint()..color = const Color(0x6670C1B3);
    final eyePaint = Paint()..color = Colors.black;
    final cheekPaint = Paint()..color = const Color(0xFFFFC1CC);
    final mouthPaintThick = Paint()
      ..color = Colors.black
      ..style = PaintingStyle.stroke
      ..strokeWidth = 4.0;
    final headPaint = Paint()..color = Colors.grey.shade600;
    final handPaint = Paint()..color = Colors.grey.shade600;
    final nozzlePaint = Paint()..color = Colors.black;
    final triggerPaint = Paint()..color = Colors.grey.shade800;

    canvas.save();
    canvas.scale(scaleX, scaleY);

    final centerX = 125.0;
    final bottomY = 243.0;

    // 병 Path → 분무기 모양 외곽 Path
    final path = Path();
    path.moveTo(100, 0); // 분사구 시작
    path.quadraticBezierTo(125, 0, 150, 0); // 분사구 끝
    path.quadraticBezierTo(170, 0, 180, 25); // 아래로
    path.quadraticBezierTo(125, 25, 70, 25); // 다시 왼쪽
    path.quadraticBezierTo(80, 0, 100, 0);
    path.close(); // 분사구 닫기

    path.moveTo(70, 25); // 몸통 시작
    path.lineTo(180, 25);
    path.quadraticBezierTo(190, 60, 190, 100);
    path.lineTo(190, 230);
    path.quadraticBezierTo(190, 245, 175, 245);
    path.lineTo(75, 245);
    path.quadraticBezierTo(60, 245, 60, 230);
    path.lineTo(60, 100);
    path.quadraticBezierTo(60, 60, 70, 25);
    path.close();

    // 병 회색 채우기
    canvas.drawPath(path, backgroundPaint);

    final handHead = Path();
    handHead.moveTo(100, -40); // 분사구 시작
    handHead.lineTo(150, -40); // 분사구 끝
    handHead.lineTo(150, 0); // 아래로
    handHead.lineTo(100, 0);
    handHead.close();
    canvas.drawPath(handHead, handPaint);

    // 트리거 손잡이
    final trigger = Path();
    trigger.moveTo(160, -35);
    trigger.quadraticBezierTo(150, -15, 170, -5);
    trigger.quadraticBezierTo(180, -15, 170, -35);
    canvas.drawPath(trigger, triggerPaint);

    final sprayerHead = Path();
    // 분사기 윗부분 헤드
    sprayerHead.moveTo(70, -50);
    sprayerHead.quadraticBezierTo(110, -70, 180, -60);
    sprayerHead.lineTo(190, -50);
    sprayerHead.lineTo(180, -30);
    sprayerHead.quadraticBezierTo(120, -40, 70, -30);
    sprayerHead.close();
    canvas.drawPath(sprayerHead, headPaint);

    // 노즐
    final nozzle = Path();
    nozzle.moveTo(190, -50);
    nozzle.lineTo(210, -48);
    nozzle.lineTo(210, -32);
    nozzle.lineTo(190, -30);
    nozzle.close();
    canvas.drawPath(nozzle, nozzlePaint);

    // 병 내부 wave 클리핑
    canvas.save();
    canvas.clipPath(path);
    final waveTop = bottomY * (1.0 - level);
    _drawWave(canvas, const Size(250, 250), wavePaint1, animation.value, waveTop, 10.0);
    _drawWave(canvas, const Size(250, 250), wavePaint2, animation.value + 0.5, waveTop, 14.0);
    canvas.restore();

    // 볼터치
    final cheekY = 250 * 0.48;
    canvas.drawOval(
      Rect.fromCenter(center: Offset(250 * 0.30, cheekY), width: 20, height: 12),
      cheekPaint,
    );
    canvas.drawOval(
      Rect.fromCenter(center: Offset(250 * 0.70, cheekY), width: 20, height: 12),
      cheekPaint,
    );

    // 눈
    final eyeY = 250 * 0.45;
    canvas.drawCircle(Offset(250 * 0.35, eyeY), 6, eyePaint);
    canvas.drawCircle(Offset(250 * 0.65, eyeY), 6, eyePaint);

    // 입
    final centerY = 250 * 0.53;
    final mouthPath = Path();
    if (level >= 0.7) {
      mouthPath.moveTo(centerX - 10, centerY);
      mouthPath.quadraticBezierTo(centerX, centerY + 6, centerX + 10, centerY);
      canvas.drawPath(mouthPath, mouthPaintThick);

    } else if (level <= 0.3) {
      mouthPath.moveTo(centerX - 12, centerY + 6);
      mouthPath.quadraticBezierTo(centerX, centerY - 4, centerX + 12, centerY + 6);
      canvas.drawPath(mouthPath, mouthPaintThick);

      // 눈물 (애니메이션으로 아래로 떨어짐)
      final tearPaint = Paint()..color = Colors.blueAccent;
      final tearDrop = (animation.value % 1.0) * 20; // 0~20까지 올라감

      // 왼쪽 눈물
      final tearLeftX = size.width * 0.365;
      final tearLeftY = centerY - 10 + tearDrop;
      if (tearDrop < 10) {
        canvas.drawOval(
          Rect.fromCenter(center: Offset(tearLeftX, tearLeftY), width: 6, height: 10),
          tearPaint,
        );
      }

      // 오른쪽 눈물 (0.5초 뒤 시차)
      final tearRightProgress = ((animation.value + 0.5) % 0.75);
      final tearRightDrop = tearRightProgress * 20;
      final tearRightX = size.width * 0.68;
      final tearRightY = centerY - 10 + tearRightDrop;

      if (tearRightDrop < 10) {
        canvas.drawOval(
          Rect.fromCenter(center: Offset(tearRightX, tearRightY), width: 6, height: 10),
          tearPaint,
        );
      }
    } else {
      canvas.drawLine(
        Offset(centerX - 10, centerY + 4),
        Offset(centerX + 10, centerY + 4),
        mouthPaintThick,
      );
    }
    canvas.restore(); // 스케일 해제
  }

  void _drawWave(Canvas canvas, Size size, Paint paint, double animValue, double baseY, double height) {
    final path = Path();
    final width = size.width;

    path.moveTo(0, baseY);
    for (double x = 0; x <= width; x++) {
      final waveY = baseY + height * sin((2 * pi / width) * x + (2 * pi * animValue));
      path.lineTo(x, waveY);
    }
    path.lineTo(width, size.height);
    path.lineTo(0, size.height);
    path.close();
    canvas.drawPath(path, paint);
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}

class CuteTrashWidget extends StatefulWidget {
  final double level;
  final Animation<double> animation;

  const CuteTrashWidget({
    super.key,
    required this.level,
    required this.animation,
  });

  @override
  State<CuteTrashWidget> createState() => _CuteTrashWidgetState();
}

class _CuteTrashWidgetState extends State<CuteTrashWidget> {
  ui.Image? trashImage;
  int trashLevelStage = 1;

  @override
  void initState() {
    super.initState();
    trashLevelStage = _getStage(widget.level);
    _loadTrashImage(trashLevelStage);
  }

  @override
  void didUpdateWidget(covariant CuteTrashWidget oldWidget) {
    super.didUpdateWidget(oldWidget);
    final newStage = _getStage(widget.level);
    if (newStage != trashLevelStage) {
      trashLevelStage = newStage;
      _loadTrashImage(trashLevelStage);
    }
  }

  int _getStage(double level) {
    if (level <= 0.2) return 1;
    if (level <= 0.4) return 2;
    if (level <= 0.6) return 3;
    if (level <= 0.8) return 4;
    return 5;
  }

  Future<void> _loadTrashImage(int stage) async {
    final assetName = 'assets/images/trashbin_$stage.jpg';
    final data = await rootBundle.load(assetName);
    final codec = await ui.instantiateImageCodec(data.buffer.asUint8List());
    final frame = await codec.getNextFrame();

    setState(() {
      trashImage = frame.image;
    });
  }

  @override
  Widget build(BuildContext context) {
    return CustomPaint(
      size: const Size(320, 280),
      painter: _CuteTrashPainter(
        widget.level,
        widget.animation,
        trashImage,
      ),
    );
  }
}

class _CuteTrashPainter extends CustomPainter {
  final double level;
  final Animation<double> animation;
  final ui.Image? trashImage;

  _CuteTrashPainter(this.level, this.animation, this.trashImage) : super(repaint: animation);

  @override
  void paint(Canvas canvas, Size size) {
    if (trashImage != null) {
      final double imgW = trashImage!.width.toDouble();
      final double imgH = trashImage!.height.toDouble();
      const double scale = 0.3;

      final double scaledW = imgW * scale;
      final double scaledH = imgH * scale;

      // 기준 bottom 위치: trashbin_1이 중앙일 때 bottom
      // 이미지 크기는 모두 가로 800으로 같다고 가정
      const double referenceHeight = 800.0; // trashbin_1의 원본 세로 크기 (예: 800 x 1000이면 1000)
      const double referenceScaledH = referenceHeight * scale;
      final double referenceCenterY = size.height / 2;
      final double targetBottom = referenceCenterY + referenceScaledH / 2;

      // 위치 조정: 하단 기준
      final double dx = (size.width - scaledW) / 2;
      final double dy = targetBottom - scaledH;

      canvas.drawImageRect(
        trashImage!,
        Rect.fromLTWH(0, 0, imgW, imgH),
        Rect.fromLTWH(dx, dy, scaledW, scaledH),
        Paint(),
      );
    }

    // 얼굴 요소: 눈, 볼터치, 입, 눈물
    final width = size.width;
    final height = size.height;
    final centerX = width * 0.5;
    final cheekPaint = Paint()..color = const Color(0xFFFFC1CC);
    final eyePaint = Paint()..color = Colors.black;
    final mouthPaintThick = Paint()
      ..color = Colors.black
      ..style = PaintingStyle.stroke
      ..strokeWidth = 4.0;

    // 볼터치 위치 (더 모음)
    final cheekY = height * 0.58;
    canvas.drawOval(Rect.fromCenter(center: Offset(width * 0.38, cheekY), width: 20, height: 12), cheekPaint);
    canvas.drawOval(Rect.fromCenter(center: Offset(width * 0.62, cheekY), width: 20, height: 12), cheekPaint);

    // 눈 위치 (더 모음)
    final eyeY = height * 0.55;
    canvas.drawCircle(Offset(width * 0.40, eyeY), 6, eyePaint);
    canvas.drawCircle(Offset(width * 0.60, eyeY), 6, eyePaint);

    // 입 & 눈물
    final centerY = height * 0.63;
    final mouthPath = Path();

    if (level > 0.7) {
      // 😢 찡그린 입
      mouthPath.moveTo(centerX - 10, centerY + 6);
      mouthPath.quadraticBezierTo(centerX, centerY - 2, centerX + 10, centerY + 6);
      canvas.drawPath(mouthPath, mouthPaintThick);

      final tearPaint = Paint()..color = Colors.blueAccent;
      final tearDrop = (animation.value % 1.0) * 20;

      if (tearDrop < 10) {
        canvas.drawOval(Rect.fromCenter(center: Offset(width * 0.4, centerY - 10 + tearDrop), width: 6, height: 10), tearPaint);
      }

      final tearRightProgress = ((animation.value + 0.5) % 0.75);
      final tearRightDrop = tearRightProgress * 20;
      if (tearRightDrop < 10) {
        canvas.drawOval(Rect.fromCenter(center: Offset(width * 0.6, centerY - 10 + tearRightDrop), width: 6, height: 10), tearPaint);
      }

    } else if (level <= 0.3) {
      // 😄 웃는 입 (아래로 볼록)
      mouthPath.moveTo(centerX - 12, centerY - 4);
      mouthPath.quadraticBezierTo(centerX, centerY + 6, centerX + 12, centerY - 4);
      canvas.drawPath(mouthPath, mouthPaintThick);
  } else {
      // 😐 무표정
      canvas.drawLine(
        Offset(centerX - 10, centerY + 4),
        Offset(centerX + 10, centerY + 4),
        mouthPaintThick,
      );
    }
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}

class CuteMopWidget extends StatefulWidget {
  final double level;                 // 0.0 ~ 10.0
  final Animation<double> animation;  // 눈물 애니메이션 재사용
  final VoidCallback? onFill;         // "채움" 버튼 콜백

  const CuteMopWidget({
    super.key,
    required this.level,
    required this.animation,
    this.onFill,
  });

  @override
  State<CuteMopWidget> createState() => _CuteMopWidgetState();
}

class _CuteMopWidgetState extends State<CuteMopWidget> {
  ui.Image? towelImage;
  int mopCount = 0;

  @override
  void initState() {
    super.initState();
    mopCount = widget.level.clamp(0, 10).toInt();
    _loadTowelImage(mopCount);
  }

  @override
  void didUpdateWidget(covariant CuteMopWidget oldWidget) {
    super.didUpdateWidget(oldWidget);
    final newMopCount = widget.level.clamp(0, 10).toInt();
    if (newMopCount != mopCount) {
      mopCount = newMopCount;
      _loadTowelImage(mopCount);
    }
  }

  Future<void> _loadTowelImage(int count) async {
    String? assetName;
    if (count >= 1 && count <= 2) {
      assetName = 'assets/images/towel_1.jpg';
    } else if (count >= 3 && count <= 5) {
      assetName = 'assets/images/towel_2.jpg';
    } else if (count >= 6 && count <= 7) {
      assetName = 'assets/images/towel_3.jpg';
    } else if (count >= 8 && count <= 10) {
      assetName = 'assets/images/towel_4.jpg';
    }

    if (assetName == null) {
      setState(() => towelImage = null);
      return;
    }

    final data = await rootBundle.load(assetName);
    final codec = await ui.instantiateImageCodec(data.buffer.asUint8List());
    final frame = await codec.getNextFrame();
    setState(() => towelImage = frame.image);
  }

  @override
  Widget build(BuildContext context) {
    return CustomPaint(
      size: const Size(320, 280),
      painter: _CuteMopPainter(
        (widget.level.clamp(0, 10)).toDouble(),
        widget.animation,
        towelImage,
      ),
    );
  }
}

class _CuteMopPainter extends CustomPainter {
  final double mopCount;
  final Animation<double> animation;
  final ui.Image? towelImage;

  _CuteMopPainter(this.mopCount, this.animation, this.towelImage)
      : super(repaint: animation);

  @override
  void paint(Canvas canvas, Size size) {
    if (towelImage == null) return;

    final imgW = towelImage!.width.toDouble();
    final imgH = towelImage!.height.toDouble();

    const double targetWidth = 240.0;
    final double scale = targetWidth / imgW;
    final double scaledW = imgW * scale;
    final double scaledH = imgH * scale;

    final double dx = (size.width - scaledW) / 2;
    final double dy = (size.height - scaledH) / 2 + 30;

    canvas.drawImageRect(
      towelImage!,
      Rect.fromLTWH(0, 0, imgW, imgH),
      Rect.fromLTWH(dx, dy, scaledW, scaledH),
      Paint(),
    );

    final cx = dx + scaledW / 2;
    final cy = dy + scaledH / 2;

    final cheekPaint = Paint()..color = const Color(0xFFFFC1CC);
    final eyePaint = Paint()..color = Colors.black;
    final mouthPaint = Paint()
      ..color = Colors.black
      ..style = PaintingStyle.stroke
      ..strokeWidth = 4.0;

    final leftEye = Offset(cx - 60, cy);
    final rightEye = Offset(cx - 10, cy);

    // 볼터치
    canvas.drawOval(
      Rect.fromCenter(center: leftEye.translate(-10, 10), width: 16, height: 10),
      cheekPaint,
    );
    canvas.drawOval(
      Rect.fromCenter(center: rightEye.translate(10, 10), width: 16, height: 10),
      cheekPaint,
    );

    // 눈
    canvas.drawCircle(leftEye, 6, eyePaint);
    canvas.drawCircle(rightEye, 6, eyePaint);

    // 입
    final mouthCenter = Offset((leftEye.dx + rightEye.dx) / 2, cy + 15);
    final mouthPath = Path();

    if (mopCount >= 6) {
      mouthPath.moveTo(mouthCenter.dx - 10, mouthCenter.dy);
      mouthPath.quadraticBezierTo(
        mouthCenter.dx,
        mouthCenter.dy + 8,
        mouthCenter.dx + 10,
        mouthCenter.dy,
      );
    } else if (mopCount <= 2) {
      mouthPath.moveTo(mouthCenter.dx - 12, mouthCenter.dy);
      mouthPath.quadraticBezierTo(
        mouthCenter.dx,
        mouthCenter.dy - 6,
        mouthCenter.dx + 12,
        mouthCenter.dy,
      );

      final tearPaint = Paint()..color = Colors.blueAccent;
      final tearDrop = (animation.value % 1.0) * 20;
      final tearRightProgress = ((animation.value + 0.5) % 1.0);
      final tearRightDrop = tearRightProgress * 20;
      const double tearOffsetY = 15.0;

      if (tearDrop < 18) {
        canvas.drawOval(
          Rect.fromCenter(
            center: leftEye.translate(0, tearOffsetY + tearDrop),
            width: 6,
            height: 10,
          ),
          tearPaint,
        );
      }
      if (tearRightDrop < 18) {
        canvas.drawOval(
          Rect.fromCenter(
            center: rightEye.translate(0, tearOffsetY + tearRightDrop),
            width: 6,
            height: 10,
          ),
          tearPaint,
        );
      }
    } else {
      canvas.drawLine(
        Offset(mouthCenter.dx - 10, mouthCenter.dy),
        Offset(mouthCenter.dx + 10, mouthCenter.dy),
        mouthPaint,
      );
    }

    canvas.drawPath(mouthPath, mouthPaint);
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}