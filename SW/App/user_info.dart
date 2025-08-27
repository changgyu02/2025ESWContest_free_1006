import 'package:flutter/material.dart';
import 'package:google_fonts/google_fonts.dart';
import 'package:http/http.dart' as http;
import 'dart:convert';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:project_bnw/User/login.dart';

class EditUserInfoPage extends StatefulWidget {
  const EditUserInfoPage({super.key});

  @override
  State<EditUserInfoPage> createState() => _EditUserInfoPageState();
}

class _EditUserInfoPageState extends State<EditUserInfoPage> {
  String? userId;
  final Color mainColor = const Color(0xFF70C1B3);
  bool isEditingInfo = true;
  final TextStyle labelTextStyle = GoogleFonts.notoSansKr(
    fontSize: 14,
    color: Colors.grey[700],
  );

  // 회원 정보
  final TextEditingController emailIdController = TextEditingController(); // 이메일 아이디
  final TextEditingController customDomainController = TextEditingController(); // 직접 입력 도메인
  String selectedDomain = 'naver.com'; // 선택된 도메인
  bool isCustomDomain = false; // 직접 입력 여부
  bool showSuccessMessage = false;
  double successOpacity = 1.0;
  String successMessageText = '';
  final List<String> domainOptions = ['naver.com', 'gmail.com', 'daum.net', 'hanmail.net', '직접 입력'];
  final TextEditingController nameController = TextEditingController();
  final TextEditingController genderController = TextEditingController();
  final TextEditingController currentPasswordController = TextEditingController();
  final TextEditingController newPasswordController = TextEditingController();
  final TextEditingController confirmPasswordController = TextEditingController();

  @override
  void initState() {
    super.initState();
    _loadUserInfo();
  }

  Future<void> _loadUserInfo() async {
    final prefs = await SharedPreferences.getInstance();
    userId = prefs.getString('_id');
    if (userId == null) return;

    final res = await http.get(
      Uri.parse('https://9d061e0bf84b.ngrok-free.app/userinfo?id=$userId'),
    );

    if (res.statusCode == 200) {
      final data = jsonDecode(res.body);
      final emailParts = (data['email'] ?? '').split('@');
      setState(() {
        emailIdController.text = emailParts[0];
        selectedDomain = emailParts.length > 1 ? emailParts[1] : 'naver.com';
        nameController.text = data['name'] ?? '';
        genderController.text = data['gender'] ?? '';
      });
    }
  }

  Widget _buildReadOnlyDisplayField(String label, String value) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        TextField(
          enabled: false,
          controller: TextEditingController(text: value),
          style: GoogleFonts.notoSansKr(
            color: Colors.black,
          ),
          decoration: InputDecoration(
            labelText: label,
            floatingLabelBehavior: FloatingLabelBehavior.auto,
            labelStyle: labelTextStyle,
            filled: true,
            fillColor: const Color(0xFFF5F5F5),
            border: OutlineInputBorder(
              borderRadius: BorderRadius.circular(12),
              borderSide: BorderSide.none,
            ),
          ),
        ),
        const SizedBox(height: 12),
      ],
    );
  }

  Widget _buildInputField(String label, TextEditingController controller, {bool obscure = false}) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        TextField(
          controller: controller,
          obscureText: obscure,
          style: GoogleFonts.notoSansKr(color: Colors.black),
          decoration: InputDecoration(
            labelText: label,
            floatingLabelBehavior: FloatingLabelBehavior.auto, // ✅ 겹치기 스타일
            labelStyle: labelTextStyle, // ✅ 스타일 통일
            filled: true,
            fillColor: const Color(0xFFF5F5F5),
            border: OutlineInputBorder(
              borderRadius: BorderRadius.circular(12),
              borderSide: BorderSide.none,
            ),
          ),
        ),
        const SizedBox(height: 12),
      ],
    );
  }

  Widget _buildSectionTitle(String title) {
    return Text(
      title,
      style: GoogleFonts.notoSansKr(
        fontSize: 20,
        fontWeight: FontWeight.bold,
        color: mainColor,
      ),
    );
  }

  Widget _buildEditableField(TextEditingController controller, String label) {
    return TextField(
      controller: controller,
      style: GoogleFonts.notoSansKr(color: Colors.black),
      decoration: InputDecoration(
        labelText: label,
        floatingLabelBehavior: FloatingLabelBehavior.auto,
        labelStyle: labelTextStyle,
        filled: true,
        fillColor: const Color(0xFFF5F5F5),
        border: OutlineInputBorder(
          borderRadius: BorderRadius.circular(12),
          borderSide: BorderSide.none,
        ),
      ),
    );
  }

  Widget _buildEmailDomainField() {
    final List<String> domainOptions = ['naver.com', 'gmail.com', 'daum.net', 'hanmail.net', '직접 입력'];

    // 현재 선택된 도메인이 목록에 없다면 임시로 추가
    final List<String> domainItems = List.from(domainOptions);
    if (!domainItems.contains(selectedDomain)) {
      domainItems.insert(0, selectedDomain); // 앞에 삽입
    }

    if (isCustomDomain) {
      return TextField(
        controller: customDomainController,
        style: GoogleFonts.notoSansKr(color: Colors.black),
        decoration: InputDecoration(
          labelText: '도메인',
          floatingLabelBehavior: FloatingLabelBehavior.auto,
          labelStyle: labelTextStyle,
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
                  customDomainController.text = '';
                } else {
                  isCustomDomain = false;
                  selectedDomain = value;
                  customDomainController.text = value;
                }
              });
            },
            itemBuilder: (context) => domainItems
                .map((domain) => PopupMenuItem<String>(
              value: domain,
              child: Text(domain, style: GoogleFonts.notoSansKr()),
            ))
                .toList(),
          ),
        ),
      );
    } else {
      return DropdownButtonFormField<String>(
        value: selectedDomain,
        onChanged: (value) {
          setState(() {
            if (value == '직접 입력') {
              isCustomDomain = true;
              customDomainController.text = '';
            } else {
              selectedDomain = value!;
              customDomainController.text = value;
            }
          });
        },
        decoration: InputDecoration(
          labelText: '도메인',
          floatingLabelBehavior: FloatingLabelBehavior.auto,
          labelStyle: labelTextStyle,
          filled: true,
          fillColor: const Color(0xFFF5F5F5),
          border: OutlineInputBorder(
            borderRadius: BorderRadius.circular(12),
            borderSide: BorderSide.none,
          ),
        ),
        items: domainItems.map((domain) {
          return DropdownMenuItem<String>(
            value: domain,
            child: Text(domain, style: GoogleFonts.notoSansKr()),
          );
        }).toList(),
      );
    }
  }

  Widget _buildEmailInput() {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Row(
          children: [
            Expanded(
              flex: 3,
              child: _buildEditableField(emailIdController, '이메일'),
            ),
            const SizedBox(width: 8),
            const Text('@'),
            const SizedBox(width: 8),
            Expanded(
              flex: 3,
              child: _buildEmailDomainField(), // 여기만 바뀜
            ),
          ],
        ),
        const SizedBox(height: 12),
      ],
    );
  }

  Widget _buildGenderSelection() {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        const SizedBox(height: 0.5),
        Row(
          children: [
            Expanded(
              child: RadioListTile<String>(
                contentPadding: EdgeInsets.zero,
                title: Text("남성", style: GoogleFonts.notoSansKr(fontSize: 16)),
                value: '남성',
                groupValue: genderController.text,
                onChanged: (value) {
                  setState(() {
                    genderController.text = value!;
                  });
                },
                activeColor: mainColor,
              ),
            ),
            Expanded(
              child: RadioListTile<String>(
                contentPadding: EdgeInsets.zero,
                title: Text("여성", style: GoogleFonts.notoSansKr(fontSize: 16)),
                value: '여성',
                groupValue: genderController.text,
                onChanged: (value) {
                  setState(() {
                    genderController.text = value!;
                  });
                },
                activeColor: mainColor,
              ),
            ),
          ],
        ),
        const SizedBox(height: 12),
      ],
    );
  }

  Widget _buildPasswordField(String label, TextEditingController controller) {
    return Padding(
      padding: const EdgeInsets.only(bottom: 12.0),
      child: TextFormField(
        controller: controller,
        obscureText: true,
        decoration: InputDecoration(
          labelText: label,
          labelStyle: labelTextStyle,
          floatingLabelBehavior: FloatingLabelBehavior.auto,
          filled: true,
          fillColor: const Color(0xFFF5F5F5),
          contentPadding: const EdgeInsets.symmetric(horizontal: 16, vertical: 20),
          border: OutlineInputBorder(
            borderRadius: BorderRadius.circular(12),
            borderSide: BorderSide.none,
          ),
        ),
        style: GoogleFonts.notoSansKr(fontSize: 16),
      ),
    );
  }

  void _submitEditInfo() async {
    final domain = customDomainController.text.trim();
    final email = '${emailIdController.text.trim()}@$domain';
    final response = await http.patch(
      Uri.parse('https://9d061e0bf84b.ngrok-free.app/update_info'),
      headers: {'Content-Type': 'application/json'},
      body: jsonEncode({
        'id': userId,
        'email': email,
        'name': nameController.text.trim(),
        'gender': genderController.text.trim(),
      }),
    );

    if (!mounted) return;

    if (response.statusCode == 200) {
      setState(() {
        showSuccessMessage = true;
        successOpacity = 1.0;
      });

      // 2초 후 천천히 사라지게
      Future.delayed(const Duration(seconds: 2), () {
        if (!mounted) return;
        setState(() {
          successOpacity = 0.0;
        });
      });

      // 3초 후 완전히 제거
      Future.delayed(const Duration(seconds: 3), () {
        if (!mounted) return;
        setState(() {
          showSuccessMessage = false;
        });
      });
    }
    if (response.statusCode == 200) {
      setState(() {
        successMessageText = "회원정보가 성공적으로 수정되었어요!";
        showSuccessMessage = true;
        successOpacity = 1.0;
      });

      Future.delayed(const Duration(seconds: 2), () {
        if (!mounted) return;
        setState(() {
          successOpacity = 0.0;
        });
      });

      Future.delayed(const Duration(seconds: 3), () {
        if (!mounted) return;
        setState(() {
          showSuccessMessage = false;
        });
      });
    }
  }

  void _submitEditPassword() async {
    // 1) 새 비밀번호와 확인이 일치하지 않을 때
    if (newPasswordController.text != confirmPasswordController.text) {
      _showAnimatedMessage("비밀번호 확인과 일치하지 않습니다.");
      return;
    }

    final response = await http.patch(
      Uri.parse('https://9d061e0bf84b.ngrok-free.app/update_password'),
      headers: {'Content-Type': 'application/json'},
      body: jsonEncode({
        'id': userId,
        'current_password': currentPasswordController.text.trim(),
        'new_password': newPasswordController.text.trim(),
      }),
    );

    if (!mounted) return;

    if (response.statusCode == 200) {
      _showAnimatedMessage("비밀번호가 성공적으로 변경되었어요!");
    } else {
      // 2) 현재 비밀번호가 틀렸을 경우 처리
      final body = jsonDecode(response.body);
      final error = body['detail'] ?? '';

      if (error.contains('현재 비밀번호')) {
        _showAnimatedMessage("현재 비밀번호가 일치하지 않습니다.");
      } else {
        _showAnimatedMessage("비밀번호 변경에 실패했습니다.");
      }
    }
  }

  void _showAnimatedMessage(String message) {
    setState(() {
      successMessageText = message;
      showSuccessMessage = true;
      successOpacity = 1.0;
    });

    Future.delayed(const Duration(seconds: 2), () {
      if (!mounted) return;
      setState(() {
        successOpacity = 0.0;
      });
    });

    Future.delayed(const Duration(seconds: 3), () {
      if (!mounted) return;
      setState(() {
        showSuccessMessage = false;
      });
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: const Color(0xFFF9F9F9),
      body: Stack(
        children: [
          Center(
            child: SingleChildScrollView(
              padding: const EdgeInsets.all(24.0),
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  Image.asset('assets/images/robot.png', width: 80, height: 80),
                  const SizedBox(height: 8),
                  _buildSectionTitle(isEditingInfo ? "회원정보 수정" : "비밀번호 수정"),
                  const SizedBox(height: 24),
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
                        if (isEditingInfo) ...[
                          _buildReadOnlyDisplayField('아이디', userId ?? ''),
                          _buildEmailInput(),
                          _buildInputField("이름", nameController),
                          _buildGenderSelection(),
                          const SizedBox(height: 4),
                          ElevatedButton(
                            onPressed: _submitEditInfo,
                            style: ElevatedButton.styleFrom(
                              backgroundColor: mainColor,
                              minimumSize: const Size(double.infinity, 48),
                              shape: RoundedRectangleBorder(
                                borderRadius: BorderRadius.circular(12),
                              ),
                            ),
                            child: Text("회원정보 수정", style: GoogleFonts.notoSansKr(color: Colors.white, fontWeight: FontWeight.bold)),
                          ),
                          const SizedBox(height: 12),
                          ElevatedButton(
                            onPressed: () async {
                              final navigator = Navigator.of(context);

                              final prefs = await SharedPreferences.getInstance();
                              await prefs.setBool('isLoggedIn', false);
                              await prefs.remove('_id');

                              navigator.pushReplacement(
                                MaterialPageRoute(builder: (_) => const LoginPage()),
                              );
                            },
                            style: ElevatedButton.styleFrom(
                              backgroundColor: mainColor,
                              minimumSize: const Size(double.infinity, 48),
                              shape: RoundedRectangleBorder(
                                borderRadius: BorderRadius.circular(12),
                              ),
                            ),
                            child: Text(
                              "로그아웃",
                              style: GoogleFonts.notoSansKr(
                                color: Colors.white,
                                fontWeight: FontWeight.bold,
                              ),
                            ),
                          ),
                        ] else ...[
                          _buildPasswordField("현재 비밀번호", currentPasswordController),
                          _buildPasswordField("새 비밀번호", newPasswordController),
                          _buildPasswordField("새 비밀번호 확인", confirmPasswordController),
                          const SizedBox(height: 16),
                          ElevatedButton(
                            onPressed: _submitEditPassword,
                            style: ElevatedButton.styleFrom(
                              backgroundColor: mainColor,
                              minimumSize: const Size(double.infinity, 48),
                              shape: RoundedRectangleBorder(
                                borderRadius: BorderRadius.circular(12),
                              ),
                            ),
                            child: Text("비밀번호 수정", style: GoogleFonts.notoSansKr(color: Colors.white, fontWeight: FontWeight.bold)),
                          ),
                        ]
                      ],
                    ),
                  ),
                  const SizedBox(height: 20),
                  Row(
                    children: [
                      Expanded(
                        child: ElevatedButton(
                          onPressed: () => setState(() => isEditingInfo = true),
                          style: ElevatedButton.styleFrom(
                            backgroundColor: isEditingInfo ? mainColor : Colors.grey[300],
                            shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
                            minimumSize: const Size(double.infinity, 48),
                          ),
                          child: Text("회원정보 수정", style: GoogleFonts.notoSansKr(color: Colors.white, fontWeight: FontWeight.bold)),
                        ),
                      ),
                      const SizedBox(width: 16),
                      Expanded(
                        child: ElevatedButton(
                          onPressed: () => setState(() => isEditingInfo = false),
                          style: ElevatedButton.styleFrom(
                            backgroundColor: !isEditingInfo ? mainColor : Colors.grey[300],
                            shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
                            minimumSize: const Size(double.infinity, 48),
                          ),
                          child: Text("비밀번호 수정", style: GoogleFonts.notoSansKr(color: Colors.white, fontWeight: FontWeight.bold)),
                        ),
                      ),
                    ],
                  ),
                ],
              ),
            ),
          ),

          // 하단 고정 메시지
          Positioned(
            bottom: 150,
            left: 0,
            right: 0,
            child: AnimatedOpacity(
              opacity: showSuccessMessage ? successOpacity : 0.0,
              duration: const Duration(milliseconds: 500),
              child: Center(
                child: Container(
                  padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 10),
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
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      Image.asset('assets/images/robot.png', width: 24, height: 24),
                      const SizedBox(width: 8),
                      Text(
                        successMessageText,
                        style: GoogleFonts.notoSansKr(
                          fontSize: 14,
                          color: const Color(0xFF00796B),
                          fontWeight: FontWeight.w600,
                        ),
                      ),
                    ],
                  ),
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
  }