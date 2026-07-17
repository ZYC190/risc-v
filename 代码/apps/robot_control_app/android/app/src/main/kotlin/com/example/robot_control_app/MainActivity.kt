package com.example.robot_control_app

import android.Manifest
import android.content.Intent
import android.content.pm.PackageManager
import android.media.MediaRecorder
import android.net.Uri
import android.os.Build
import android.os.SystemClock
import android.telephony.SmsManager
import android.telephony.SubscriptionManager
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import io.flutter.embedding.android.FlutterActivity
import io.flutter.embedding.engine.FlutterEngine
import io.flutter.plugin.common.MethodChannel
import java.io.File

class MainActivity : FlutterActivity() {
    private val channelName = "robot_control_app/native"
    private val preferencesName = "family_guardian_settings"
    private val smsPermissionRequestCode = 1902
    private val audioPermissionRequestCode = 1903
    private var pendingSmsPhone = ""
    private var pendingSmsBody = ""
    private var pendingSmsResult: MethodChannel.Result? = null
    private var pendingAudioPermissionResult: MethodChannel.Result? = null
    private var voiceRecorder: MediaRecorder? = null
    private var voiceFile: File? = null
    private var voiceStartedAt = 0L

    override fun configureFlutterEngine(flutterEngine: FlutterEngine) {
        super.configureFlutterEngine(flutterEngine)
        MethodChannel(
            flutterEngine.dartExecutor.binaryMessenger,
            channelName
        ).setMethodCallHandler { call, result ->
            when (call.method) {
                "loadPreferences" -> {
                    val preferences = getSharedPreferences(preferencesName, MODE_PRIVATE)
                    result.success(preferences.all)
                }
                "setPreference" -> {
                    val key = call.argument<String>("key") ?: ""
                    val value = call.argument<Any>("value")
                    if (key.isBlank()) {
                        result.error("PREFERENCE_KEY_EMPTY", "设置键不能为空", null)
                        return@setMethodCallHandler
                    }
                    val editor = getSharedPreferences(preferencesName, MODE_PRIVATE).edit()
                    when (value) {
                        is String -> editor.putString(key, value)
                        is Boolean -> editor.putBoolean(key, value)
                        is Int -> editor.putInt(key, value)
                        is Long -> editor.putLong(key, value)
                        is Double -> editor.putString(key, value.toString())
                        null -> editor.remove(key)
                        else -> editor.putString(key, value.toString())
                    }
                    editor.apply()
                    result.success(true)
                }
                "removePreference" -> {
                    val key = call.argument<String>("key") ?: ""
                    getSharedPreferences(preferencesName, MODE_PRIVATE)
                        .edit()
                        .remove(key)
                        .apply()
                    result.success(true)
                }
                "openSms" -> {
                    val phone = call.argument<String>("phone") ?: ""
                    val body = call.argument<String>("body") ?: ""
                    try {
                        val intent = Intent(Intent.ACTION_SENDTO).apply {
                            data = Uri.parse("smsto:$phone")
                            putExtra("sms_body", body)
                        }
                        startActivity(intent)
                        result.success(true)
                    } catch (e: Exception) {
                        result.error("SMS_ERROR", e.message, null)
                    }
                }
                "openUrl" -> {
                    val url = call.argument<String>("url") ?: ""
                    try {
                        val intent = Intent(Intent.ACTION_VIEW, Uri.parse(url))
                        startActivity(intent)
                        result.success(true)
                    } catch (e: Exception) {
                        result.error("URL_ERROR", e.message, null)
                    }
                }
                "sendSms" -> {
                    val phone = call.argument<String>("phone") ?: ""
                    val body = call.argument<String>("body") ?: ""
                    if (phone.isBlank()) {
                        result.error("SMS_PHONE_EMPTY", "短信号码不能为空", null)
                        return@setMethodCallHandler
                    }
                    val missingPermissions = requiredSmsPermissions().filter {
                        ContextCompat.checkSelfPermission(this, it) !=
                            PackageManager.PERMISSION_GRANTED
                    }
                    if (missingPermissions.isNotEmpty()) {
                        pendingSmsPhone = phone
                        pendingSmsBody = body
                        pendingSmsResult = result
                        ActivityCompat.requestPermissions(
                            this,
                            missingPermissions.toTypedArray(),
                            smsPermissionRequestCode
                        )
                    } else {
                        sendSmsNow(phone, body, result)
                    }
                }
                "startVoiceRecording" -> startVoiceRecording(result)
                "stopVoiceRecording" -> stopVoiceRecording(result)
                "cancelVoiceRecording" -> {
                    releaseVoiceRecorder(deleteFile = true)
                    result.success(true)
                }
                else -> result.notImplemented()
            }
        }
    }

    private fun requiredSmsPermissions(): Array<String> {
        return arrayOf(Manifest.permission.SEND_SMS)
    }

    private fun preferredSubscriptionId(): Int {
        var fallback = SubscriptionManager.getDefaultSmsSubscriptionId()
        if (ContextCompat.checkSelfPermission(
                this,
                Manifest.permission.READ_PHONE_STATE
            ) != PackageManager.PERMISSION_GRANTED
        ) {
            return fallback
        }

        return try {
            val subscriptionManager = getSystemService(SubscriptionManager::class.java)
            val subscriptions = subscriptionManager.activeSubscriptionInfoList ?: emptyList()
            if (subscriptions.any { it.subscriptionId == fallback }) {
                fallback
            } else {
                subscriptions.firstOrNull()?.subscriptionId ?: fallback
            }
        } catch (_: Exception) {
            fallback
        }
    }

    private fun sendSmsNow(phone: String, body: String, result: MethodChannel.Result) {
        try {
            val subscriptionId = preferredSubscriptionId()
            val smsManager = if (
                Build.VERSION.SDK_INT >= Build.VERSION_CODES.S &&
                    subscriptionId != SubscriptionManager.INVALID_SUBSCRIPTION_ID
            ) {
                getSystemService(SmsManager::class.java)
                    .createForSubscriptionId(subscriptionId)
            } else if (subscriptionId != SubscriptionManager.INVALID_SUBSCRIPTION_ID) {
                @Suppress("DEPRECATION")
                SmsManager.getSmsManagerForSubscriptionId(subscriptionId)
            } else {
                @Suppress("DEPRECATION")
                SmsManager.getDefault()
            }
            val parts = smsManager.divideMessage(body)
            if (parts.size > 1) {
                smsManager.sendMultipartTextMessage(phone, null, parts, null, null)
            } else {
                smsManager.sendTextMessage(phone, null, body, null, null)
            }
            result.success(true)
        } catch (e: Exception) {
            result.error("SMS_SEND_ERROR", e.message, null)
        }
    }

    private fun startVoiceRecording(result: MethodChannel.Result) {
        if (voiceRecorder != null) {
            result.error("AUDIO_ALREADY_RECORDING", "已经在录音", null)
            return
        }
        if (
            ContextCompat.checkSelfPermission(this, Manifest.permission.RECORD_AUDIO) !=
                PackageManager.PERMISSION_GRANTED
        ) {
            pendingAudioPermissionResult = result
            ActivityCompat.requestPermissions(
                this,
                arrayOf(Manifest.permission.RECORD_AUDIO),
                audioPermissionRequestCode
            )
            return
        }

        try {
            val output = File(cacheDir, "parent_voice_${System.currentTimeMillis()}.m4a")
            @Suppress("DEPRECATION")
            val recorder = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                MediaRecorder(this)
            } else {
                MediaRecorder()
            }
            recorder.setAudioSource(MediaRecorder.AudioSource.MIC)
            recorder.setOutputFormat(MediaRecorder.OutputFormat.MPEG_4)
            recorder.setAudioEncoder(MediaRecorder.AudioEncoder.AAC)
            recorder.setAudioChannels(1)
            recorder.setAudioSamplingRate(16000)
            recorder.setAudioEncodingBitRate(48000)
            // Flutter stops and sends at 15 s; this is a final native guard.
            recorder.setMaxDuration(17000)
            recorder.setOutputFile(output.absolutePath)
            recorder.prepare()
            recorder.start()
            voiceRecorder = recorder
            voiceFile = output
            voiceStartedAt = SystemClock.elapsedRealtime()
            result.success(true)
        } catch (e: Exception) {
            releaseVoiceRecorder(deleteFile = true)
            result.error("AUDIO_START_ERROR", e.message, null)
        }
    }

    private fun stopVoiceRecording(result: MethodChannel.Result) {
        val recorder = voiceRecorder
        val output = voiceFile
        if (recorder == null || output == null) {
            result.error("AUDIO_NOT_RECORDING", "当前没有录音", null)
            return
        }
        val durationMs = SystemClock.elapsedRealtime() - voiceStartedAt
        try {
            recorder.stop()
            recorder.release()
            voiceRecorder = null
            voiceFile = null
            voiceStartedAt = 0L
            val bytes = output.readBytes()
            output.delete()
            if (durationMs < 500 || bytes.isEmpty()) {
                result.error("AUDIO_TOO_SHORT", "录音时间太短", null)
                return
            }
            result.success(
                mapOf(
                    "audio" to bytes,
                    "durationMs" to durationMs,
                    "mime" to "audio/mp4"
                )
            )
        } catch (e: Exception) {
            releaseVoiceRecorder(deleteFile = true)
            result.error("AUDIO_STOP_ERROR", e.message, null)
        }
    }

    private fun releaseVoiceRecorder(deleteFile: Boolean) {
        try {
            voiceRecorder?.stop()
        } catch (_: Exception) {
        }
        try {
            voiceRecorder?.release()
        } catch (_: Exception) {
        }
        voiceRecorder = null
        voiceStartedAt = 0L
        if (deleteFile) {
            voiceFile?.delete()
        }
        voiceFile = null
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<out String>,
        grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == smsPermissionRequestCode) {
            val result = pendingSmsResult
            if (result == null) {
                return
            }
            if (
                grantResults.isNotEmpty() &&
                    grantResults.all { it == PackageManager.PERMISSION_GRANTED }
            ) {
                sendSmsNow(pendingSmsPhone, pendingSmsBody, result)
            } else {
                result.error("SMS_PERMISSION_DENIED", "未授予短信或电话状态权限", null)
            }
            pendingSmsPhone = ""
            pendingSmsBody = ""
            pendingSmsResult = null
        } else if (requestCode == audioPermissionRequestCode) {
            val result = pendingAudioPermissionResult ?: return
            pendingAudioPermissionResult = null
            if (
                grantResults.isNotEmpty() &&
                    grantResults.all { it == PackageManager.PERMISSION_GRANTED }
            ) {
                startVoiceRecording(result)
            } else {
                result.error("AUDIO_PERMISSION_DENIED", "未授予麦克风权限", null)
            }
        }
    }

    override fun onDestroy() {
        releaseVoiceRecorder(deleteFile = true)
        super.onDestroy()
    }
}
