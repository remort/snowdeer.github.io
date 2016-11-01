---
layout: post
title: SQLite (외부 DB 파일 사용) 예제
category: Android
tag: [android, sqlite]
---

안드로이드 SQLite 예제 코드입니다. 이번에는 [SQLiteManager](https://addons.mozilla.org/ko/firefox/addon/sqlite-manager/)와 
같은 외부 Tool을 이용해서 미리 만들어 놓은 DB 파일과 연동하는 방법입니다.

저는 외부 Tool을 이용하는 것을 선호합니다. 
수정 등이 쉽고, 별도로 DB 파일에 접속해서 데이터들을 눈으로 확인하기도 쉽기 때문입니다.

안드로이드에서 제공하는 SQLite Helper 클래스를 조금 수정하여, 
DB 파일과 연동할 수 있도록 해보겠습니다.


<pre class="prettyprint" style="font-size:0.7em;">
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

import android.content.Context;
import android.database.SQLException;
import android.database.sqlite.SQLiteDatabase;
import android.database.sqlite.SQLiteOpenHelper;

public class SnowFileDBOpenHelper extends SQLiteOpenHelper {

    private SQLiteDatabase sqlite;

    private final Context mContext;
    private final String mFolderPath;
    private final static String mDBFileName = "snowdeer_db.sqlite";

    public SnowFileDBOpenHelper(Context context) {
        super(context, mDBFileName, null, 1);

        mContext = context;
        // Eclipse
		mFolderPath = Environment.getExternalStoragePublicDirectory(null) 
                + "/Android/data/" + context.getPackageName() + "/";

        // Android Studio
        mFolderPath = mContext.getExternalFilesDir(null) + "/";

    }

    @Override
    public void onCreate(SQLiteDatabase db) {
    }

    @Override
    public void onUpgrade(SQLiteDatabase db, int oldVersion, int newVersion) {
    }

    public void checkDatabase() {
        try {
            checkFolderExist();
            checkFileExist();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void checkFolderExist() throws IOException {
        File targetFolder = new File(mFolderPath);
        if (targetFolder.exists() == false) {
            targetFolder.mkdirs();
        }
    }

    private void checkFileExist() throws IOException {
        File targetFile = new File(mFolderPath + mDBFileName);
        if (targetFile.exists() == false) {
            copyDatabase();
        }
    }

    private void copyDatabase() throws IOException {
        InputStream inputStream = mContext.getAssets().open(mDBFileName);

        String outFileName = mFolderPath + mDBFileName;
        OutputStream outputStream = new FileOutputStream(outFileName);

        byte[] buffer = new byte[1024];
        int length;

        while ((length = inputStream.read(buffer)) > 0) {
            outputStream.write(buffer, 0, length);
        }

        outputStream.flush();
        outputStream.close();
        inputStream.close();
    }

    public void openDataBase() throws SQLException {
        String myPath = mFolderPath + mDBFileName;
        sqlite = SQLiteDatabase.openDatabase(myPath, null,
                SQLiteDatabase.NO_LOCALIZED_COLLATORS);
    }

    @Override
    public synchronized void close() {
        if (sqlite != null) {
            sqlite.close();
        }

        super.close();
    }

    @Override
    public SQLiteDatabase getReadableDatabase() {
        checkDatabase();
        openDataBase();

        return sqlite;
    }

    @Override
    public SQLiteDatabase getWritableDatabase() {
        checkDatabase();
        openDataBase();

        return sqlite;
    }
}
</pre>

그리고 DB Manager 클래스의 예제는 다음과 같습니다.

<pre class="prettyprint" style="font-size:0.7em;">
import java.util.ArrayList;
import android.content.ContentValues;
import android.content.Context;
import android.database.Cursor;
import android.database.sqlite.SQLiteDatabase;

public class SnowFileDBQueryManager {

    private static final String TABLE_SNOW = "TABLE_SNOW";

    private static SnowFileDBQueryManager mInstance
            = new SnowFileDBQueryManager();

    private SnowFileDBQueryManager() {
    }

    public static SnowFileDBQueryManager getInstance() {
        return mInstance;
    }

    public ArrayList<SnowItem> getSnowItemList(Context context) {
        ArrayList<SnowItem> resultList = new ArrayList<SnowItem>();

        SnowFileDBOpenHelper dbHelper = new SnowFileDBOpenHelper(context);
        try {
            String query = "SELECT _id, userInfo, address FROM "
                    + TABLE_SNOW;

            SQLiteDatabase db = dbHelper.getReadableDatabase();
            Cursor cursor = db.rawQuery(query, null);

            while (cursor.moveToNext()) {
                SnowItem item = new SnowItem(cursor.getInt(0),
                        cursor.getString(1),
                        cursor.getString(2));
                
                resultList.add(item);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }

        dbHelper.close();
        return resultList;
    }

    public void addSnowItem(Context context, SnowItem item) {
        SnowFileDBOpenHelper dbHelper = new SnowFileDBOpenHelper(context);

        try {
            SQLiteDatabase db = dbHelper.getWritableDatabase();
            ContentValues row = new ContentValues();

            int id = 0;
            if (item.id > 0) {
                id = item.id;
            } else {
                id = getMaxSnowItemId(context);
            }

            row.put("_id", id);
            row.put("userInfo", item.userInfo);
            row.put("address", item.address);

            db.insert(TABLE_SNOW, null, row);
        } catch (Exception e) {
            e.printStackTrace();
        }
        dbHelper.close();
    }

    public void updateSnowItem(Context context, SnowItem item) {
        SnowFileDBOpenHelper dbHelper = new SnowFileDBOpenHelper(context);

        try {
            SQLiteDatabase db = dbHelper.getWritableDatabase();
            ContentValues row = new ContentValues();

            int id = 0;
            if (item.id > 0) {
                id = item.id;
            } else {
                id = getMaxSnowItemId(context);
            }

            row.put("_id", id);
            row.put("userInfoId", item.userInfo);
            row.put("address", item.address);

            String strFilter = "_id = " + item.id;
            db.update(TABLE_SNOW, row, strFilter, null);

        } catch (Exception e) {
            e.printStackTrace();
        }
        dbHelper.close();
    }

    public void deleteSnowItem(Context context, SnowItem item) {
        SnowFileDBOpenHelper dbHelper = new SnowFileDBOpenHelper(context);

        try {
            SQLiteDatabase db = dbHelper.getWritableDatabase();
            String strFilter = "_id = " + item.id;

            db.delete(TABLE_SNOW, strFilter, null);
        } catch (Exception e) {
            e.printStackTrace();
        }
        dbHelper.close();
    }

    private int getMaxSnowItemId(Context context) {
        int result = 0;
        SnowFileDBOpenHelper dbHelper = new SnowFileDBOpenHelper(context);
        try {
            SQLiteDatabase db = dbHelper.getReadableDatabase();
            Cursor cursor = db.rawQuery("SELECT MAX(_id) FROM " 
                    + TABLE_SNOW, null);

            while (cursor.moveToNext()) {
                result = cursor.getInt(0);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }

        dbHelper.close();
        result = result + 1;

        return result;
    }
}
</pre>
