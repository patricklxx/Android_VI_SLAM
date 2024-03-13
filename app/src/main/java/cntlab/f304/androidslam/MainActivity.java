package cntlab.f304.androidslam;

import android.Manifest;
import android.app.NativeActivity;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.os.Environment;

import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

public class MainActivity extends NativeActivity
{
    @Override
    protected void onCreate(Bundle state)
    {
        super.onCreate(state);

        verifyStoragePermission(this);

        int check = ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA);
        if(check == PackageManager.PERMISSION_DENIED)
        {
            String[] required_list = new String[] { Manifest.permission.CAMERA };
            ActivityCompat.requestPermissions(this, required_list, 5);
        }

        setContentView(R.layout.activity_main);
    }

    private static final int REQUEST_EXTERNAL_STORAGE = 1;
    private static String[] PERMISSIONS_STORAGE = {
            "android.permission.READ_EXTERNAL_STORAGE",
            "android.permission.WRITE_EXTERNAL_STORAGE",
            "android.permission.MANAGE_EXTERNAL_STORAGE"
    };
    public void verifyStoragePermission(NativeActivity activity){
        try{
            int permission = ActivityCompat.checkSelfPermission(activity,"android.permission.WRITE_EXTERNAL_STORAGE");
            if(permission!= PackageManager.PERMISSION_GRANTED){
                ActivityCompat.requestPermissions(activity,PERMISSIONS_STORAGE, REQUEST_EXTERNAL_STORAGE);
            }
        }catch (Exception e){
            e.printStackTrace();
        }
    }

}