ESP8266WebServer web_server(80);

void webServerSendFile(String filename) {
  File f = SPIFFS.open("/" + filename, "r");
  String data = f.readString();

  web_server.send(200, "text/html", data);
}

void handleRoot() {
  webServerSendFile("index.html");
}

void a() {
  nana.setDriveMode(1);
}

void left() {
  nana.drive(-4095, 4095);

  web_server.send(200, "text/html", "ok");
}

void right() {
  nana.drive(4095, -4095);

  web_server.send(200, "text/html", "ok");
}

void up() {
  nana.drive(4095, 4095);

  web_server.send(200, "text/html", "ok");
}

void down() {
  nana.drive(-4095, -4095);

  web_server.send(200, "text/html", "ok");
}

void space() {
  nana.drive(0, 0);
  nana.moving_mode = nana.WAITING;

  web_server.send(200, "text/html", "ok");
}

void web_setup() {
  web_server.on("/", handleRoot);

  web_server.on("/32", space);

  web_server.on("/37", left);
  web_server.on("/38", up);
  web_server.on("/39", right);
  web_server.on("/40", down);

  web_server.on("/65", a);

  web_server.begin();
}
