---
layout: post
title: Simple Web Server
category: Java
tag: [java]
---
# Simple Web Server

<pre class="prettyprint">
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import com.sun.net.httpserver.HttpServer;

import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;

public class SimpleWebServer {
    public static void main(String[] args) throws Exception {
        System.out.println("Hello. Simple Web Server.");

        HttpServer server = HttpServer.create(new InetSocketAddress(8080), 0);
        server.createContext("/", indexHandler);
        server.createContext("/test", mHandler);
        server.setExecutor(null);
        server.start();
    }

    private static HttpHandler indexHandler = new HttpHandler() {

        @Override
        public void handle(HttpExchange httpExchange) throws IOException {
            byte[] response = "This is an index page.".getBytes();
            httpExchange.sendResponseHeaders(200, response.length);
            OutputStream os = httpExchange.getResponseBody();
            os.write(response);
            os.close();
        }
    };

    private static HttpHandler mHandler = new HttpHandler() {

        @Override
        public void handle(HttpExchange httpExchange) throws IOException {
            byte[] response = "This is a test page.".getBytes();
            httpExchange.sendResponseHeaders(200, response.length);
            OutputStream os = httpExchange.getResponseBody();
            os.write(response);
            os.close();
        }
    };
}
</pre>