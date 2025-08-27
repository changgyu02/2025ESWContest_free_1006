import 'package:flutter/material.dart';
import 'package:google_fonts/google_fonts.dart';

class NotificationLogPage extends StatefulWidget {
  final List<Map<String, String>> logs;

  const NotificationLogPage({super.key, required this.logs});

  @override
  State<NotificationLogPage> createState() => _NotificationLogPageState();
}

class _NotificationLogPageState extends State<NotificationLogPage> {
  final GlobalKey<AnimatedListState> _listKey = GlobalKey<AnimatedListState>();
  late List<Map<String, String>> recentLogs;

  @override
  void initState() {
    super.initState();
    recentLogs = widget.logs
        .where((log) => _isRecent(log['time']))
        .toList();
  }

  bool _isRecent(String? timeStr) {
    if (timeStr == null) return false;
    final parsed = DateTime.tryParse(timeStr);
    if (parsed == null) return false;
    return DateTime.now().difference(parsed).inMinutes <= 60;
  }

  String _getImage(String title) {
    if (title.contains("배터리")) return 'assets/images/battery.png';
    if (title.contains("알코올")) return 'assets/images/alcohol.png';
    if (title.contains("쓰레기")) return 'assets/images/garbage.png';
    if (title.contains("걸레")) return 'assets/images/mop.png';
    return 'assets/images/robot.png';
  }

  Widget _buildItemWithoutDismissible(Map<String, String> log) {
    final title = log['title'] ?? '';
    final body = log['body'] ?? '';
    final time = log['time'] ?? '';
    final imgPath = _getImage(title);

    return Container(
      margin: const EdgeInsets.only(bottom: 16),
      padding: const EdgeInsets.all(14),
      decoration: BoxDecoration(
        color: const Color(0xFFE0F2F1),
        borderRadius: BorderRadius.circular(20),
        boxShadow: const [
          BoxShadow(
            color: Colors.black12,
            blurRadius: 6,
            offset: Offset(0, 3),
          ),
        ],
      ),
      child: Row(
        children: [
          Image.asset(imgPath, width: 40, height: 40),
          const SizedBox(width: 12),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  title,
                  style: GoogleFonts.notoSansKr(
                    fontSize: 16,
                    fontWeight: FontWeight.bold,
                    color: const Color(0xFF00796B),
                  ),
                ),
                const SizedBox(height: 4),
                Text(
                  body,
                  style: GoogleFonts.notoSansKr(
                    fontSize: 14,
                    color: Colors.grey[800],
                  ),
                ),
              ],
            ),
          ),
          const SizedBox(width: 8),
          Text(
            time.split('.')[0].replaceFirst('T', '\n'),
            style: GoogleFonts.notoSansKr(
              fontSize: 12,
              color: Colors.grey[600],
            ),
            textAlign: TextAlign.right,
          ),
        ],
      ),
    );
  }

  Widget _buildAnimatedItem(Map<String, String> log, int index, Animation<double> animation) {
    return SizeTransition(
      sizeFactor: animation,
      axisAlignment: 0.0,
      child: Dismissible(
        key: UniqueKey(),
        direction: DismissDirection.endToStart,
        background: Container(
          alignment: Alignment.centerRight,
          padding: const EdgeInsets.symmetric(horizontal: 20),
          decoration: BoxDecoration(
            color: Colors.redAccent,
            borderRadius: BorderRadius.circular(20),
          ),
          child: const Icon(Icons.delete, color: Colors.white),
        ),
        onDismissed: (_) {
          final removedItem = log;
          final removeIndex = recentLogs.indexOf(removedItem);
          if (removeIndex == -1) return;

          _listKey.currentState?.removeItem(
            removeIndex,
                (context, animation) => _buildItemWithoutDismissible(removedItem),
            duration: Duration.zero,
          );

          setState(() {
            recentLogs.removeAt(removeIndex);
            widget.logs.remove(removedItem);
          });
        },
        child: _buildItemWithoutDismissible(log),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    final Color mainColor = const Color(0xFF70C1B3);

    return Scaffold(
      backgroundColor: const Color(0xFFF9F9F9),
      appBar: AppBar(
        backgroundColor: Colors.white,
        title: Text(
          "알림 내역",
          style: GoogleFonts.notoSansKr(
            fontWeight: FontWeight.bold,
            color: mainColor,
          ),
        ),
        centerTitle: true,
        elevation: 1,
        iconTheme: const IconThemeData(color: Colors.black87),
      ),
      body: recentLogs.isEmpty
          ? Center(
        child: Text(
          "최근 1시간 내 알림이 없습니다.",
          style: GoogleFonts.notoSansKr(fontSize: 17, color: Colors.grey),
        ),
      )
          : AnimatedList(
        key: _listKey,
        padding: const EdgeInsets.all(16),
        initialItemCount: recentLogs.length,
        itemBuilder: (context, index, animation) =>
            _buildAnimatedItem(recentLogs[index], index, animation),
      ),
    );
  }
}