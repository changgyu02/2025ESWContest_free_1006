import 'dart:io';

void log(String message) {
  stdout.writeln(message); // print 대신 사용
}

void main() async {
  final directory = Directory('./lib'); // lib 디렉토리 탐색
  const oldBaseUrl = 'https://9457-192-203-145-55.ngrok-free.app';
  const newBaseUrl = 'https://9d061e0bf84b.ngrok-free.app ';

  // 모든 .dart 파일 탐색
  await for (var file in directory.list(recursive: true)) {
    if (file is File && file.path.endsWith('.dart')) {
      final content = await file.readAsString();

      if (content.contains(oldBaseUrl)) {
        final updatedContent = content.replaceAll(oldBaseUrl, newBaseUrl);

        try {
          await file.writeAsString(updatedContent);
          log('Updated: ${file.path}');
        } catch (e) {
          log('Failed to update: ${file.path}, Error: $e');
        }
      } else {
        log('No match found in: ${file.path}');
      }
    }
  }

  log('URL replacement complete.');
}