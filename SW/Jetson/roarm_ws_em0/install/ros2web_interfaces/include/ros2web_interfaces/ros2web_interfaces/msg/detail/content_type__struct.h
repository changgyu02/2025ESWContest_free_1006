// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2web_interfaces:msg/ContentType.idl
// generated code does not contain a copyright notice

#ifndef ROS2WEB_INTERFACES__MSG__DETAIL__CONTENT_TYPE__STRUCT_H_
#define ROS2WEB_INTERFACES__MSG__DETAIL__CONTENT_TYPE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'APPLICATION_JSON'.
/**
  * application
 */
static const char * const ros2web_interfaces__msg__ContentType__APPLICATION_JSON = "application/json";

/// Constant 'APPLICATION_MSWORD'.
static const char * const ros2web_interfaces__msg__ContentType__APPLICATION_MSWORD = "application/msword";

/// Constant 'APPLICATION_MSWORD_OPEN_XML'.
static const char * const ros2web_interfaces__msg__ContentType__APPLICATION_MSWORD_OPEN_XML = "application/vnd.openxmlformats-officedocument.wordprocessingml.document";

/// Constant 'APPLICATION_OCTET_STREAM'.
static const char * const ros2web_interfaces__msg__ContentType__APPLICATION_OCTET_STREAM = "application/octet-stream";

/// Constant 'APPLICATION_VND_AMAZON_EBOOK'.
static const char * const ros2web_interfaces__msg__ContentType__APPLICATION_VND_AMAZON_EBOOK = "application/vnd.amazon.ebook";

/// Constant 'APPLICATION_X_ABIWORD'.
static const char * const ros2web_interfaces__msg__ContentType__APPLICATION_X_ABIWORD = "application/x-abiword";

/// Constant 'APPLICATION_X_BZIP2'.
static const char * const ros2web_interfaces__msg__ContentType__APPLICATION_X_BZIP2 = "application/x-bzip2";

/// Constant 'APPLICATION_X_BZIP'.
static const char * const ros2web_interfaces__msg__ContentType__APPLICATION_X_BZIP = "application/x-bzip";

/// Constant 'APPLICATION_X_CSH'.
static const char * const ros2web_interfaces__msg__ContentType__APPLICATION_X_CSH = "application/x-csh";

/// Constant 'APPLICATION_X_FREEARC'.
static const char * const ros2web_interfaces__msg__ContentType__APPLICATION_X_FREEARC = "application/x-freearc";

/// Constant 'APPLICATION_X_WWW_FORM_URLENCODED'.
static const char * const ros2web_interfaces__msg__ContentType__APPLICATION_X_WWW_FORM_URLENCODED = "application/x-www-form-urlencoded";

/// Constant 'AUDIO_AAC'.
/**
  * application/epub+zip
  * application/gzip
  * application/java-archive
  * application/ld+json
  * application/ogg
  * application/pdf
  * application/rtf
  * application/vnd.apple.installer+xml
  * application/vnd.mozilla.xul+xml
  * application/vnd.ms-excel
  * application/vnd.ms-fontobject
  * application/vnd.ms-powerpoint
  * application/vnd.oasis.opendocument.presentation
  * application/vnd.oasis.opendocument.spreadsheet
  * application/vnd.oasis.opendocument.text
  * application/vnd.openxmlformats-officedocument.presentationml.presentation
  * application/vnd.rar
  * application/vnd.visio
  * application/x-7z-compressed
  * application/x-httpd-php
  * application/x-sh
  * application/x-shockwave-flash
  * application/x-tar
  * application/xhtml+xml
  * application/xml
  * application/zip
  * audio
 */
static const char * const ros2web_interfaces__msg__ContentType__AUDIO_AAC = "audio/aac";

/// Constant 'IMAGE_BMP'.
/**
  * audio/3gpp
  * audio/3gpp2
  * audio/midi
  * audio/mpeg
  * audio/ogg
  * audio/opus
  * audio/wav
  * audio/webm
  * audio/x-midi
  * font
  * font/otf
  * font/ttf
  * font/woff
  * font/woff2
  * image
 */
static const char * const ros2web_interfaces__msg__ContentType__IMAGE_BMP = "image/bmp";

/// Constant 'IMAGE_JPEG'.
static const char * const ros2web_interfaces__msg__ContentType__IMAGE_JPEG = "image/jpeg";

/// Constant 'IMAGE_PNG'.
static const char * const ros2web_interfaces__msg__ContentType__IMAGE_PNG = "image/png";

/// Constant 'MULTIPART_FORM_DATA'.
/**
  * image/gif
  * image/svg+xml
  * image/tiff
  * image/vnd.microsoft.icon
  * image/webp
 */
static const char * const ros2web_interfaces__msg__ContentType__MULTIPART_FORM_DATA = "multipart/form-data";

/// Constant 'TEXT_CSS'.
/**
  * pplication/vnd.openxmlformats-officedocument.spreadsheetml.sheet
  * text
 */
static const char * const ros2web_interfaces__msg__ContentType__TEXT_CSS = "text/css";

/// Constant 'TEXT_CSV'.
static const char * const ros2web_interfaces__msg__ContentType__TEXT_CSV = "text/csv";

/// Constant 'TEXT_HTML'.
static const char * const ros2web_interfaces__msg__ContentType__TEXT_HTML = "text/html";

/// Constant 'TEXT_JAVASCRIPT'.
static const char * const ros2web_interfaces__msg__ContentType__TEXT_JAVASCRIPT = "text/javascript";

/// Constant 'TEXT_PLAIN'.
static const char * const ros2web_interfaces__msg__ContentType__TEXT_PLAIN = "text/plain";

/// Constant 'TEXT_XML'.
static const char * const ros2web_interfaces__msg__ContentType__TEXT_XML = "text/xml";

/// Constant 'VIDEO_X_MSVIDEO'.
/**
  * text/calendar
  * video
 */
static const char * const ros2web_interfaces__msg__ContentType__VIDEO_X_MSVIDEO = "video/x-msvideo";

/// Struct defined in msg/ContentType in the package ros2web_interfaces.
/**
  * https://developer.mozilla.org/ja/docs/Web/HTTP/Basics_of_HTTP/MIME_types/Common_types
 */
typedef struct ros2web_interfaces__msg__ContentType
{
  uint8_t structure_needs_at_least_one_member;
} ros2web_interfaces__msg__ContentType;

// Struct for a sequence of ros2web_interfaces__msg__ContentType.
typedef struct ros2web_interfaces__msg__ContentType__Sequence
{
  ros2web_interfaces__msg__ContentType * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2web_interfaces__msg__ContentType__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2WEB_INTERFACES__MSG__DETAIL__CONTENT_TYPE__STRUCT_H_
