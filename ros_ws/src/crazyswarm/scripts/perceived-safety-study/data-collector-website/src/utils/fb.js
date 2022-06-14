// Import the functions you need from the SDKs you need
import { initializeApp } from "firebase/app";
import { getFirestore } from "firebase/firestore";
// TODO: Add SDKs for Firebase products that you want to use
// https://firebase.google.com/docs/web/setup#available-libraries

// Your web app's Firebase configuration
const firebaseConfig = {
  apiKey: "AIzaSyA5LQ6BPqr6uWJwmO90iZaTRirLS3rmQa4",
  authDomain: "data-collector-website.firebaseapp.com",
  projectId: "data-collector-website",
  storageBucket: "data-collector-website.appspot.com",
  messagingSenderId: "603203336002",
  appId: "1:603203336002:web:ac2b6fb16ca677b41bfadb",
};

// Initialize Firebase
const app = initializeApp(firebaseConfig);

export const db = getFirestore(app);
