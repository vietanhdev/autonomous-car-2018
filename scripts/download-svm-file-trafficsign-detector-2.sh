echo "Downloading SVM file for Traffic Sign Detector 2"
mkdir -p tmp
cd tmp
../scripts/download-gdrive.sh "https://drive.google.com/uc?export=download&id=1V7ptTyn_GdoHeyRHdYEhlkjneCoAp6xO" "svm_trafficsign_legacy.zip"
cd ..

echo "Extracing ZIP file"
unzip tmp/svm_trafficsign_legacy.zip -d main_ws/src/team806/data/