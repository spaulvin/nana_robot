ESP8266WebServer web_server(80);

void webServerSendFile(String filename) {
  File f = SPIFFS.open("/" + filename, "r");
  String data = f.readString();

  web_server.send(200, "text/html", data);
}

void handleRoot() {
  webServerSendFile("index.html");
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

  web_server.send(200, "text/html", "ok");
}

void show_map() {
  String room_map = "";
  for (int i = 0; i < nana.map_size ; i++) {
    for (int j = 0; j < nana.map_size ; j++) {
      room_map += String(nana.room_map[i][j]);
    }
    room_map += "<br>";
  }
  web_server.send(200, "text/html", room_map);
}

void web_setup() {
  web_server.on("/", handleRoot);

  web_server.on("/32", space);

  web_server.on("/37", left);
  web_server.on("/38", up);
  web_server.on("/39", right);
  web_server.on("/40", down);

  web_server.on("/map", show_map);

  web_server.begin();
}
