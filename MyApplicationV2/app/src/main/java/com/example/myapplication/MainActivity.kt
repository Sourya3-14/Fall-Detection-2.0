package com.example.myapplication

import android.app.Activity
import android.os.Bundle
import android.util.Log
import android.widget.Toast
import androidx.activity.enableEdgeToEdge
import androidx.appcompat.app.AppCompatActivity
import androidx.core.view.ViewCompat
import androidx.core.view.WindowInsetsCompat
import com.example.myapplication.databinding.ActivityMainBinding
import com.google.firebase.database.DataSnapshot
import com.google.firebase.database.DatabaseError
import com.google.firebase.database.DatabaseReference
import com.google.firebase.database.FirebaseDatabase
import com.google.firebase.database.ValueEventListener


class MainActivity : AppCompatActivity() {
    private lateinit var binding : ActivityMainBinding
    private lateinit var database : DatabaseReference
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        read3()
    }
    private fun read3(){
        database = FirebaseDatabase.getInstance().getReference("Sensor/data")

        // Read data continuously from Firebase
        database.addValueEventListener(object : ValueEventListener {
            override fun onDataChange(snapshot: DataSnapshot) {
                var pred : String = "Safe"
                var bpm : String = "72"
                var spo2 : String = "96"
                if (snapshot.exists()) {
                    val data = snapshot.getValue(String::class.java)// Read as String
                    if (data!=null) {
                        val (rest, status) = data.split(";")
                        val (x, y) = rest.split(",")
                        pred = status.substring(0,status.length-1)
                        bpm = x
                        spo2 = y
                    }
                    binding.textView6.text = pred + "!!!" // Set text to TextView5
                    Log.d("FirebaseData", "Data received: $pred")
                    binding.textView4.text = spo2 + " %"  // Set text to TextView5
                    Log.d("FirebaseData", "Data received: $spo2")
                    binding.textView5.text = bpm // Set text to TextView5
                    Log.d("FirebaseData", "Data received: $bpm")
                }
            }

            override fun onCancelled(error: DatabaseError) {
                Log.e("FirebaseError", "Failed to read data: ${error.message}")
            }
        })
    }
}