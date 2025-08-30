# https://docs.aiohttp.org/en/stable/web_reference.html

string[] headers  # [<key>, <value>, <key>, <value>, ...]
string method     # HTTP method,
string path       # The URL including PATH INFO without the host or scheme. 
string query      # The query string in the URL

string srv_name
string route

uint8[] body
string text
string content_type   # ContentType
string charset
BodyPart[] multipart
