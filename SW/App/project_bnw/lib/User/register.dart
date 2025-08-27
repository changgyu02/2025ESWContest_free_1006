import 'package:flutter/material.dart';
import 'package:google_fonts/google_fonts.dart';
import 'package:http/http.dart' as http;
import 'dart:convert';
import 'package:project_bnw/User/login.dart';

class RegisterPage extends StatefulWidget {
  const RegisterPage({super.key});

  @override
  RegisterPageState createState() => RegisterPageState();
}

class RegisterPageState extends State<RegisterPage> {
  final TextEditingController idController = TextEditingController();
  final TextEditingController nameController = TextEditingController();
  final TextEditingController emailIdController = TextEditingController();
  final TextEditingController emailDomainController = TextEditingController();
  final TextEditingController passwordController = TextEditingController();
  final TextEditingController confirmPasswordController = TextEditingController();

  String gender = '';
  bool agreedToTerms = false;
  String message = "";

  final Color mainColor = const Color(0xFF70C1B3);
  String? selectedDomain = 'naver.com';
  bool isCustomDomain = false;

  final List<String> domainOptions = [
    'naver.com',
    'gmail.com',
    'daum.net',
    'hanmail.net',
    '직접 입력',
  ];

  Future<void> _register() async {
    final id = idController.text.trim();
    final name = nameController.text.trim();
    final emailId = emailIdController.text.trim();
    final emailDomain = emailDomainController.text.trim();
    final password = passwordController.text.trim();
    final confirmPassword = confirmPasswordController.text.trim();
    final email = '$emailId@$emailDomain';

    if (id.isEmpty || name.isEmpty || emailId.isEmpty || emailDomain.isEmpty || password.isEmpty || confirmPassword.isEmpty || gender.isEmpty) {
      setState(() {
        message = "모든 항목을 입력해주세요.";
      });
      return;
    }

    if (!agreedToTerms) {
      setState(() {
        message = "약관에 동의해주세요.";
      });
      return;
    }

    if (password != confirmPassword) {
      setState(() {
        message = "비밀번호가 일치하지 않습니다.";
      });
      return;
    }

    try {
      final response = await http.post(
        Uri.parse('https://9d061e0bf84b.ngrok-free.app/register'),
        headers: {'Content-Type': 'application/json'},
        body: jsonEncode({
          "id": id,
          "name": name,
          "email": email,
          "password": password,
          "gender": gender,
        }),
      );

      if (response.statusCode == 200) {
        setState(() {
          message = "회원가입이 완료되었습니다.";
        });

        WidgetsBinding.instance.addPostFrameCallback((_) {
          Navigator.pushReplacement(
            context,
            MaterialPageRoute(builder: (context) => const LoginPage()),
          );
        });
      } else {
        setState(() {
          message = "회원가입 실패: 이미 존재하는 계정이거나 오류입니다.";
        });
      }
    } catch (e) {
      setState(() {
        message = "서버에 연결할 수 없습니다.";
      });
    }
  }

  Widget _buildEmailInput() {
    return Row(
      children: [
        // 이메일 아이디 입력 필드
        Expanded(
          flex: 5,
          child: TextField(
            controller: emailIdController,
            decoration: InputDecoration(
              hintText: '이메일 주소',
              hintStyle: GoogleFonts.notoSansKr(color: Colors.grey),
              filled: true,
              fillColor: const Color(0xFFF5F5F5),
              border: OutlineInputBorder(
                borderRadius: BorderRadius.circular(12),
                borderSide: BorderSide.none,
              ),
            ),
          ),
        ),
        const Padding(
          padding: EdgeInsets.symmetric(horizontal: 8),
          child: Text("@"),
        ),
        // 이메일 도메인 입력 필드 (드롭다운 + 텍스트 필드 혼합)
        Expanded(
          flex: 5,
          child: TextField(
            controller: emailDomainController,
            readOnly: false,
            decoration: InputDecoration(
              hintText: '도메인',
              hintStyle: GoogleFonts.notoSansKr(color: Colors.grey),
              filled: true,
              fillColor: const Color(0xFFF5F5F5),
              border: OutlineInputBorder(
                borderRadius: BorderRadius.circular(12),
                borderSide: BorderSide.none,
              ),
              suffixIcon: PopupMenuButton<String>(
                icon: const Icon(Icons.arrow_drop_down),
                onSelected: (value) {
                  setState(() {
                    if (value == '직접 입력') {
                      isCustomDomain = true;
                      emailDomainController.clear();
                    } else {
                      isCustomDomain = false;
                      emailDomainController.text = value;
                    }
                  });
                },
                itemBuilder: (context) {
                  return domainOptions.map((domain) {
                    return PopupMenuItem(
                      value: domain,
                      child: Text(domain, style: GoogleFonts.notoSansKr()),
                    );
                  }).toList();
                },
              ),
            ),
          ),
        ),
      ],
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Center(
        child: SingleChildScrollView(
          padding: const EdgeInsets.all(24.0),
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              Image.asset(
                'assets/images/robot.png',
                width: 80,
                height: 80,
              ),
              const SizedBox(height: 12),
              Text(
                "회원가입을 위해 정보를 입력해주세요.",
                style: GoogleFonts.notoSansKr(
                  fontSize: 20,
                  fontWeight: FontWeight.bold,
                  color: mainColor,
                ),
              ),
              const SizedBox(height: 32),
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
                      controller: idController,
                      decoration: InputDecoration(
                        hintText: '아이디',
                        hintStyle: GoogleFonts.notoSansKr(color: Colors.grey),
                        filled: true,
                        fillColor: const Color(0xFFF5F5F5),
                        border: OutlineInputBorder(
                          borderRadius: BorderRadius.circular(12),
                          borderSide: BorderSide.none,
                        ),
                      ),
                    ),
                    const SizedBox(height: 12),
                    _buildEmailInput(),
                    const SizedBox(height: 12),
                    TextField(
                      controller: nameController,
                      decoration: InputDecoration(
                        hintText: '이름',
                        hintStyle: GoogleFonts.notoSansKr(color: Colors.grey),
                        filled: true,
                        fillColor: const Color(0xFFF5F5F5),
                        border: OutlineInputBorder(
                          borderRadius: BorderRadius.circular(12),
                          borderSide: BorderSide.none,
                        ),
                      ),
                    ),
                    const SizedBox(height: 12),
                    TextField(
                      controller: passwordController,
                      obscureText: true,
                      decoration: InputDecoration(
                        hintText: '비밀번호',
                        hintStyle: GoogleFonts.notoSansKr(color: Colors.grey),
                        filled: true,
                        fillColor: const Color(0xFFF5F5F5),
                        border: OutlineInputBorder(
                          borderRadius: BorderRadius.circular(12),
                          borderSide: BorderSide.none,
                        ),
                      ),
                    ),
                    const SizedBox(height: 12),
                    TextField(
                      controller: confirmPasswordController,
                      obscureText: true,
                      decoration: InputDecoration(
                        hintText: '비밀번호 확인',
                        hintStyle: GoogleFonts.notoSansKr(color: Colors.grey),
                        filled: true,
                        fillColor: const Color(0xFFF5F5F5),
                        border: OutlineInputBorder(
                          borderRadius: BorderRadius.circular(12),
                          borderSide: BorderSide.none,
                        ),
                      ),
                    ),
                    const SizedBox(height: 12),
                    Row(
                      children: [
                        Expanded(
                          child: RadioListTile<String>(
                            value: "남성",
                            groupValue: gender,
                            activeColor: mainColor,
                            onChanged: (value) {
                              setState(() {
                                gender = value!;
                              });
                            },
                            title: Text("남성", style: GoogleFonts.notoSansKr()),
                            contentPadding: const EdgeInsets.symmetric(horizontal: 0),
                            visualDensity: VisualDensity.compact,
                          ),
                        ),
                        Expanded(
                          child: RadioListTile<String>(
                            value: "여성",
                            groupValue: gender,
                            activeColor: mainColor,
                            onChanged: (value) {
                              setState(() {
                                gender = value!;
                              });
                            },
                            title: Text("여성", style: GoogleFonts.notoSansKr()),
                            contentPadding: const EdgeInsets.symmetric(horizontal: 0),
                            visualDensity: VisualDensity.compact,
                          ),
                        ),
                      ],
                    ),
                    CheckboxListTile(
                      value: agreedToTerms,
                      activeColor: mainColor,
                      onChanged: (value) {
                        setState(() {
                          agreedToTerms = value ?? false;
                        });
                      },
                      title: Text(
                        "개인정보 수집 및 정보이용에 동의합니다.",
                        style: GoogleFonts.notoSansKr(fontSize: 14),
                      ),
                      controlAffinity: ListTileControlAffinity.leading,
                      contentPadding: const EdgeInsets.symmetric(horizontal: 0),
                      visualDensity: VisualDensity.compact,
                    ),
                    const SizedBox(height: 12),
                    SizedBox(
                      width: double.infinity,
                      height: 48,
                      child: ElevatedButton(
                        onPressed: _register,
                        style: ElevatedButton.styleFrom(
                          backgroundColor: mainColor,
                          shape: RoundedRectangleBorder(
                            borderRadius: BorderRadius.circular(12),
                          ),
                        ),
                        child: Text(
                          "Sign Up",
                          style: GoogleFonts.notoSansKr(
                            fontSize: 16,
                            fontWeight: FontWeight.bold,
                            color: Colors.white,
                          ),
                        ),
                      ),
                    ),
                    if (message.isNotEmpty)
                      Padding(
                        padding: const EdgeInsets.only(top: 12),
                        child: Text(
                          message,
                          style: GoogleFonts.notoSansKr(color: Colors.red),
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