---
layout: post
title: 장소 자동 완성 예제
category: Android
tag: [Android, Location]
---
장소 검색 텍스트 박스를 자동으로 완성하는 방법입니다. 구글에서 해당 기능을 쉽게 사용할 수 있도록
컴포넌트를 제공하고 있습니다. [Place Picker](/android/2017/08/21/place-autocomplete-sample/)와 
마찬가지로 'Google Places API for Android' 라이브러리에대한 [Google API Key 등록](/android/2017/01/23/android-how-to-use-google-api-key/)을 해야 합니다.

장소 자동 완성은 크게 Fragment를 사용하는 방법과 Intent 호출을 이용하는 방법의 두 가지가 있습니다.

<br>

# Fragment 사용하는 코드

<pre class="prettyprint">
public class PlaceAutocompleteActivity extends AppCompatActivity {

  static final String TAG = "PlaceAutocomplete";
  PlaceAutocompleteFragment autocompleteFragment;

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_place_autocomplete);

    autocompleteFragment = (PlaceAutocompleteFragment)
        getFragmentManager().findFragmentById(R.id.place_autocomplete_fragment);

    autocompleteFragment.setOnPlaceSelectedListener(new PlaceSelectionListener() {
      @Override
      public void onPlaceSelected(Place place) {
        Log.i(TAG, "Place: " + place.getName());
      }

      @Override
      public void onError(Status status) {
        Log.i(TAG, "An error occurred: " + status);
      }
    });
  }
}
</pre>

<br>

# Intent를 이용하는 방법


<pre class="prettyprint">
public class PlaceAutocompleteActivity extends AppCompatActivity {

  int PLACE_AUTOCOMPLETE_REQUEST_CODE = 1;

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_place_autocomplete);

    findViewById(R.id.btn_autocomplete).setOnClickListener(new OnClickListener() {
      @Override
      public void onClick(View v) {
        try {
          Intent intent = new PlaceAutocomplete.IntentBuilder(PlaceAutocomplete.MODE_FULLSCREEN)
              .build(PlaceAutocompleteActivity.this);
          startActivityForResult(intent, PLACE_AUTOCOMPLETE_REQUEST_CODE);
        } catch (GooglePlayServicesRepairableException e) {
          // TODO: Handle the error.
        } catch (GooglePlayServicesNotAvailableException e) {
          // TODO: Handle the error.
        }
      }
    });
  }

  @Override
  protected void onActivityResult(int requestCode, int resultCode, Intent data) {
    if (requestCode == PLACE_AUTOCOMPLETE_REQUEST_CODE) {
      if (resultCode == RESULT_OK) {
        Place place = PlacePicker.getPlace(data, this);
        String toastMsg = String.format("Place: %s", place.getName());
        Toast.makeText(this, toastMsg, Toast.LENGTH_LONG).show();
      }
    }
  }

}
</pre>
