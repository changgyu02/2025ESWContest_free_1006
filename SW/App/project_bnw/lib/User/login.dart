import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'dart:convert';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:google_fonts/google_fonts.dart';
import 'package:project_bnw/User/register.dart';
import 'package:project_bnw/main_home.dart';

class LoginPage extends StatefulWidget {
  const LoginPage({super.key});

  @override
  LoginPageState createState() => LoginPageState();
}

class LoginPageState extends State<LoginPage> {
  final TextEditingController _idController = TextEditingController();
  final TextEditingController _passwordController = TextEditingController();
  bool _isPasswordVisible = false;
  bool _keepLoggedIn = false;
  String _message = "";

  final Color mainColor = const Color(0xFF70C1B3); // 민트색

  @override
  void initState() {
    super.initState();
    _checkLoginStatus();
  }

  Future<void> _checkLoginStatus() async {
    final prefs = await SharedPreferences.getInstance();
    final isLoggedIn = prefs.getBool('isLoggedIn') ?? false;
    if (isLoggedIn && mounted) {
      Navigator.pushReplacement(
        context,
        MaterialPageRoute(builder: (context) => const MainHomePage()),
      );
    }
  }

  Future<void> _login() async {
    final String id = _idController.text.trim();
    final String password = _passwordController.text.trim();

    try {
      final response = await http.post(
        Uri.parse('https://9d061e0bf84b.ngrok-free.app/login'),
        headers: {'Content-Type': 'application/json'},
        body: jsonEncode({"_id": id, "password": password}),
      );

      if (response.statusCode == 200) {
        final data = jsonDecode(response.body);
        final String userId = data['_id'] ?? "";

        if (userId.isEmpty) {
          setState(() {
            _message = "서버 응답에 _id가 없습니다.";
          });
          return;
        }

        final prefs = await SharedPreferences.getInstance();
        await prefs.setString('_id', userId);
        if (_keepLoggedIn) {
          await prefs.setBool('isLoggedIn', true);
        }

        if (!mounted) return;
        Navigator.pushReplacement(
          context,
          MaterialPageRoute(builder: (context) => const MainHomePage()),
        );
      } else {
        setState(() {
          _message = "로그인 실패: ID 또는 비밀번호가 잘못되었습니다.";
        });
      }
    } catch (e) {
      debugPrint("로그인 오류: $e");
      setState(() {
        _message = "서버에 연결할 수 없습니다.";
      });
    }
  }

  void _navigateToRegisterPage() {
    Navigator.push(
      context,
      MaterialPageRoute(builder: (context) => const RegisterPage()),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: const Color(0xFFF9F9F9),
      body: Center(
        child: SingleChildScrollView(
          padding: const EdgeInsets.all(24.0),
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              // 상단 로봇 이미지 & 설명
              Image.asset(
                'assets/images/robot.png',
                width: 80,
                height: 80,
              ),
              const SizedBox(height: 12),
              Text(
                "B&W Cleaning Bot",
                style: GoogleFonts.notoSansKr(
                  fontSize: 24,
                  fontWeight: FontWeight.bold,
                  color: mainColor,
                ),
              ),
              Text(
                "테이블을 깨끗하게! 자율주행 청소 로봇 어플",
                style: GoogleFonts.notoSansKr(
                  fontSize: 15,
                  color: Colors.grey[700],
                ),
              ),
              const SizedBox(height: 32),

              // 로그인 입력 폼
              Container(
                padding: const EdgeInsets.all(20),
                decoration: BoxDecoration(
                  color: Colors.white,
                  borderRadius: BorderRadius.circular(16),
                  boxShadow: const [
                    BoxShadow(
                      color: Colors.black12,
                      blurRadius: 10,
                      offset: Offset(0, 4),
                    ),
                  ],
                ),
                child: Column(
                  children: [
                    TextField(
                      controller: _idController,
                      style: GoogleFonts.notoSansKr(),
                      decoration: InputDecoration(
                        hintText: 'ID',
                        hintStyle: GoogleFonts.notoSansKr(color: Colors.grey),
                        filled: true,
                        fillColor: const Color(0xFFF5F5F5),
                        border: OutlineInputBorder(
                          borderRadius: BorderRadius.circular(12),
                          borderSide: BorderSide.none,
                        ),
                      ),
                    ),
                    const SizedBox(height: 16),
                    TextField(
                      controller: _passwordController,
                      obscureText: !_isPasswordVisible,
                      style: GoogleFonts.notoSansKr(),
                      decoration: InputDecoration(
                        hintText: 'Password',
                        hintStyle: GoogleFonts.notoSansKr(color: Colors.grey),
                        filled: true,
                        fillColor: const Color(0xFFF5F5F5),
                        border: OutlineInputBorder(
                          borderRadius: BorderRadius.circular(12),
                          borderSide: BorderSide.none,
                        ),
                        suffixIcon: IconButton(
                          icon: Icon(
                            _isPasswordVisible
                                ? Icons.visibility
                                : Icons.visibility_off,
                          ),
                          onPressed: () {
                            setState(() {
                              _isPasswordVisible = !_isPasswordVisible;
                            });
                          },
                        ),
                      ),
                    ),
                    const SizedBox(height: 12),
                    Row(
                      children: [
                        Checkbox(
                          value: _keepLoggedIn,
                          activeColor: mainColor,
                          onChanged: (bool? value) {
                            setState(() {
                              _keepLoggedIn = value ?? false;
                            });
                          },
                        ),
                        Text(
                          "로그인 상태 유지",
                          style: GoogleFonts.notoSansKr(),
                        ),
                      ],
                    ),
                    const SizedBox(height: 12),
                    SizedBox(
                      width: double.infinity,
                      height: 48,
                      child: ElevatedButton(
                        onPressed: _login,
                        style: ElevatedButton.styleFrom(
                          backgroundColor: mainColor,
                          shape: RoundedRectangleBorder(
                            borderRadius: BorderRadius.circular(12),
                          ),
                        ),
                        child: Text(
                          "Login",
                          style: GoogleFonts.notoSansKr(
                            fontSize: 16,
                            fontWeight: FontWeight.bold,
                            color: Colors.white,
                          ),
                        ),
                      ),
                    ),
                    const SizedBox(height: 12),
                    if (_message.isNotEmpty)
                      Text(
                        _message,
                        style: GoogleFonts.notoSansKr(color: Colors.red),
                      ),
                  ],
                ),
              ),
              const SizedBox(height: 20),
              TextButton(
                onPressed: _navigateToRegisterPage,
                child: Text(
                  "아직 계정이 없나요?",
                  style: GoogleFonts.notoSansKr(
                    fontSize: 14,
                    color: Colors.grey,
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