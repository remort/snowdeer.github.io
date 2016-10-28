---
layout: post
title: SQLite 사용 예제
category: Android
tag: [android, sqlite]
---

안드로이드 SQLite 예제 코드입니다.
직접 코드를 이용해서 Database를 생성할 수도 있고,
[SQLiteManager](https://addons.mozilla.org/ko/firefox/addon/sqlite-manager/)와 같은 외부 Tool을 이용해서 Database를 생성할 수도 있습니다.

저는 Tool을 사용하는 것을 선호하긴 하지만(나중에 수정이나 유지 보수가 쉬워서),
이번 포스팅에서는 코드를 이용해서 작성하는 코드를 올려봅니다.

안드로이드에서는 SQLite를 좀 더 사용하기 쉽도록 Helper 클래스를
제공하고 있습니다. Helper 클래스를 이용하여 Database를 생성하거나
업그레이드 등을 쉽게 할 수 있습니다.

<pre class="prettyprint" style="font-size:0.7em;">
import android.content.Context;
import android.database.sqlite.SQLiteDatabase;
import android.database.sqlite.SQLiteOpenHelper;

public class SnowSQLiteOpenHelper extends SQLiteOpenHelper {
    public static final String TABLE_MY_WALLET = "TABLE_MY_WALLET";
    public static final String TABLE_BEACON_HISTORY = "TABLE_BEACON_HISTORY";

    public SnowSQLiteOpenHelper(Context context){
        super(context, "snowdeer.db", null, 1);
    }

    public void onCreate(SQLiteDatabase db){
        db.execSQL("CREATE TABLE " + TABLE_MY_WALLET
                + "(_id INTEGER PRIMARY KEY AUTOINCREMENT," +
                "privateKey TEXT, publicKey TEXT, address TEXT);");
        db.execSQL("CREATE TABLE " + TABLE_BEACON_HISTORY
                + "(_id INTEGER PRIMARY KEY AUTOINCREMENT," +
                "inputTime TEXT, beaconId TEXT, rssi INTEGER);");
    }

    public void onUpgrade(SQLiteDatabase db, int oldVersion, int newVersion){
        db.execSQL("DROP TABLE IF EXISTS " + TABLE_MY_WALLET);
        db.execSQL("DROP TABLE IF EXISTS " + TABLE_BEACON_HISTORY);

        onCreate(db);
    }
}
</pre>

그리고 SQL의 CRUD(Create, Retrieve, Update, Delete) 기능을 수행하는
Manager 클래스를 둡니다. 테이블(Table) 별로 따로 두는 것이 더 바람직하지만,
여기서는 하나의 클래스에 모두 모아놓았습니다.  
(Database 규모에 따라 모두 모아놓는 것이 편리할 때도 있습니다.)

<pre class="prettyprint" style="font-size:0.7em;">
import android.content.ContentValues;
import android.content.Context;
import android.database.Cursor;
import android.database.sqlite.SQLiteDatabase;

import com.lnc.prototype.data.BeaconLog;
import com.lnc.prototype.data.Wallet;

import java.util.ArrayList;

/**
 * Created by snowd on 2016-10-28.
 */

public class SnowDBManager {
    private static final SnowDBManager mInstance = new SnowDBManager();
    private SnowDBManager() {}

    public static SnowDBManager getInstance() {
        return mInstance;
    }

    public ArrayList<Wallet> getWalletList(Context context) {
        ArrayList<Wallet> resultList = new ArrayList<>();

        SnowSQLiteOpenHelper dbHelper = new SnowSQLiteOpenHelper(context);
        try {

            SQLiteDatabase db = dbHelper.getReadableDatabase();
            Cursor cursor = db.rawQuery(
                            "SELECT _id, privateKey, publicKey, address FROM "
                                    + SnowSQLiteOpenHelper.TABLE_MY_WALLET,
                            null);

            while (cursor.moveToNext()) {
                int id = cursor.getInt(0);
                String privateKey = cursor.getString(1);
                String publicKey = cursor.getString(2);
                String address = cursor.getString(3);

                Wallet wallet = new Wallet(id, privateKey, publicKey, address);
                resultList.add(wallet);
            }
            cursor.close();
        } catch (Exception e) {
            e.printStackTrace();
        }

        dbHelper.close();

        return resultList;
    }

    public void addWallet(Context context, Wallet item) {
        SnowSQLiteOpenHelper dbHelper = new SnowSQLiteOpenHelper(context);

        try {
            SQLiteDatabase db = dbHelper.getWritableDatabase();

            ContentValues row = new ContentValues();

            row.put("privateKey", item.privateKey);
            row.put("publicKey", item.publicKey);
            row.put("address", item.address);

            db.insert(SnowSQLiteOpenHelper.TABLE_MY_WALLET, null, row);

            db.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
        dbHelper.close();
    }

    public void updateWallet(Context context, Wallet item) {
        SnowSQLiteOpenHelper dbHelper = new SnowSQLiteOpenHelper(context);

        try {
            SQLiteDatabase db = dbHelper.getWritableDatabase();
            String filter = "_id = " + item.id;
            ContentValues row = new ContentValues();

            row.put("privateKey", item.privateKey);
            row.put("publicKey", item.publicKey);
            row.put("address", item.address);

            db.update(SnowSQLiteOpenHelper.TABLE_MY_WALLET, row, filter, null);
            db.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
        dbHelper.close();
    }

    public void deleteWallet(Context context, Wallet item) {
        SnowSQLiteOpenHelper dbHelper = new SnowSQLiteOpenHelper(context);

        try {
            SQLiteDatabase db = dbHelper.getWritableDatabase();
            String filter = "_id = " + item.id;

            db.delete(SnowSQLiteOpenHelper.TABLE_MY_WALLET, filter, null);
            db.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
        dbHelper.close();
    }

    public void clearWallet(Context context) {
        SnowSQLiteOpenHelper dbHelper = new SnowSQLiteOpenHelper(context);

        try {
            SQLiteDatabase db = dbHelper.getWritableDatabase();

            db.delete(SnowSQLiteOpenHelper.TABLE_MY_WALLET, "", null);
            db.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
        dbHelper.close();
    }

    public ArrayList<BeaconLog> getBeaconLogList(Context context) {
        ArrayList<BeaconLog> resultList = new ArrayList<>();

        SnowSQLiteOpenHelper dbHelper = new SnowSQLiteOpenHelper(context);
        try {

            SQLiteDatabase db = dbHelper.getReadableDatabase();
            Cursor cursor = db.rawQuery(
                    "SELECT _id, inputTime, beaconId, rssi FROM "
                            + SnowSQLiteOpenHelper.TABLE_BEACON_HISTORY,
                    null);

            while (cursor.moveToNext()) {
                int id = cursor.getInt(0);
                String inputTime = cursor.getString(1);
                String beaconId = cursor.getString(2);
                int rssi = cursor.getInt(3);

                BeaconLog beaconLog = new BeaconLog(id, inputTime, beaconId, rssi);
                resultList.add(beaconLog);
            }
            cursor.close();
        } catch (Exception e) {
            e.printStackTrace();
        }

        dbHelper.close();

        return resultList;
    }

    public void addBeaconLog(Context context, BeaconLog item) {
        SnowSQLiteOpenHelper dbHelper = new SnowSQLiteOpenHelper(context);

        try {
            SQLiteDatabase db = dbHelper.getWritableDatabase();

            ContentValues row = new ContentValues();

            row.put("inputTime", item.inputTime);
            row.put("beaconId", item.beaconId);
            row.put("rssi", item.rssi);

            db.insert(SnowSQLiteOpenHelper.TABLE_BEACON_HISTORY, null, row);

            db.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
        dbHelper.close();
    }

    public void updateBeaconLog(Context context, BeaconLog item) {
        SnowSQLiteOpenHelper dbHelper = new SnowSQLiteOpenHelper(context);

        try {
            SQLiteDatabase db = dbHelper.getWritableDatabase();
            String filter = "_id = " + item.id;
            ContentValues row = new ContentValues();

            row.put("inputTime", item.inputTime);
            row.put("beaconId", item.beaconId);
            row.put("rssi", item.rssi);

            db.update(SnowSQLiteOpenHelper.TABLE_BEACON_HISTORY, row, filter, null);
            db.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
        dbHelper.close();
    }

    public void deleteBeaconLog(Context context, BeaconLog item) {
        SnowSQLiteOpenHelper dbHelper = new SnowSQLiteOpenHelper(context);

        try {
            SQLiteDatabase db = dbHelper.getWritableDatabase();
            String filter = "_id = " + item.id;

            db.delete(SnowSQLiteOpenHelper.TABLE_BEACON_HISTORY, filter, null);
            db.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
        dbHelper.close();
    }

    public void clearBeaconLog(Context context) {
        SnowSQLiteOpenHelper dbHelper = new SnowSQLiteOpenHelper(context);

        try {
            SQLiteDatabase db = dbHelper.getWritableDatabase();

            db.delete(SnowSQLiteOpenHelper.TABLE_BEACON_HISTORY, "", null);
            db.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
        dbHelper.close();
    }
}
</pre>