---
layout: post
title: File에 텍스트(Text) 읽고 쓰기
category: Android
tag: [Android]
---

## File에 텍스트 읽고 쓰는 예제

</pre class="prettyprint">
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;

public class FileUtil {
  
  public synchronized static void writeTextToFile(String filepath, String text) {
    File file = new File(filepath);
    FileWriter fw = null;
    BufferedWriter bw = null;

    try {
      fw = new FileWriter(file);
      bw = new BufferedWriter(fw);
      bw.write(text);
    } catch (Exception e) {
      e.printStackTrace();
    } finally {
      try {
        bw.close();
      } catch (Exception e) {
        e.printStackTrace();
      }

      try {
        fw.close();
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }

  public synchronized static String getTextFromFile(String filepath) {
    File file = new File(filepath);

    StringBuilder sb = new StringBuilder();

    try {
      BufferedReader br = new BufferedReader(new FileReader(file));
      String line;

      while ((line = br.readLine()) != null) {
        sb.append(line);
        sb.append('\n');
      }
      br.close();
    } catch (Exception e) {
      e.printStackTrace();
    }

    return sb.toString();
  }
}
</pre>