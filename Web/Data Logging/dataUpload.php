<?php
require_once 'firebaseLib.php';
// --- This is your Firebase URL
$url = 'https://earthquake-38823.firebaseio.com/';
// --- Use your token from Firebase here
$token = 'WKWnFPBDT4D4Nx8D674w00YacvqlAKdwICWu9YRX';
// --- Here is your parameter from the http GET
//$time = $_GET['time'];
$time = time();
$data = $_GET['data'];
// --- Set up your Firebase url structure here
$firebasePathData = "log/" . $time;
$logFB = new fireBase($url, $token);
$logFB->set($firebasePathData, $data);
echo"success";
?>