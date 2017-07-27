---
layout: post
title: 내부/외부 저장소 용량 체크
category: Android
tag: [Android]
---

모바일 디바이스내 내부/외부 저장소의 용량을 확인하는 함수입니다.

<br>

# MemoryUtil.java

<pre class="prettyprint">
public class MemoryUtil {

  public static void showStatus() {
    Log.i("", "< MemoryStatus >");
    Log.i("", "Total Internal MemorySize : " + getFormattedSize(GetTotalInternalMemorySize()));
    Log.i("",
        "Available Internal MemorySize : " + getFormattedSize(GetAvailableInternalMemorySize()));

    if (IsExternalMemoryAvailable() == true) {
      Log.i("", "Total External MemorySize : " + getFormattedSize(GetTotalExternalMemorySize()));
      Log.i("",
          "Available External MemorySize : " + getFormattedSize(GetAvailableExternalMemorySize()));
    }
  }

  private static boolean IsExternalMemoryAvailable() {
    return android.os.Environment.getExternalStorageState()
        .equals(android.os.Environment.MEDIA_MOUNTED);
  }

  private static long GetTotalInternalMemorySize() {
    File path = Environment.getDataDirectory();
    StatFs stat = new StatFs(path.getPath());
    long blockSize = stat.getBlockSize();
    long totalBlocks = stat.getBlockCount();

    return totalBlocks * blockSize;
  }

  private static long GetAvailableInternalMemorySize() {
    File path = Environment.getDataDirectory();
    StatFs stat = new StatFs(path.getPath());
    long blockSize = stat.getBlockSize();
    long availableBlocks = stat.getAvailableBlocks();

    return availableBlocks * blockSize;
  }

  private static long GetTotalExternalMemorySize() {
    if (IsExternalMemoryAvailable() == true) {
      File path = Environment.getExternalStorageDirectory();
      StatFs stat = new StatFs(path.getPath());
      long blockSize = stat.getBlockSize();
      long totalBlocks = stat.getBlockCount();

      return totalBlocks * blockSize;
    }

    return -1;
  }

  private static long GetAvailableExternalMemorySize() {
    if (IsExternalMemoryAvailable() == true) {
      File path = Environment.getExternalStorageDirectory();
      StatFs stat = new StatFs(path.getPath());
      long blockSize = stat.getBlockSize();
      long availableBlocks = stat.getAvailableBlocks();

      return availableBlocks * blockSize;
    }

    return -1;
  }

  private static String getFormattedSize(long size) {
    String suffix = "";

    if (size >= 1024) {
      suffix = "KB";
      size /= 1024;
      if (size >= 1024) {
        suffix = "MB";
        size /= 1024;
      }
    }

    StringBuilder sb = new StringBuilder(Long.toString(size));

    int commaOffset = sb.length() - 3;
    while (commaOffset > 0) {
      sb.insert(commaOffset, ',');
      commaOffset -= 3;
    }

    sb.append(suffix);

    return sb.toString();
  }
}
</pre>