echo "Downloading Simulator 4"
mkdir -p tmp
cd tmp
../scripts/download-gdrive.sh "https://drive.google.com/uc?export=download&id=12E4KJzIQAAppZHnX557dwhWJPx5zyaJB" "simulator4.zip"
cd ..

echo "Extracing ZIP file"
mkdir -p simulators
unzip tmp/simulator4.zip -d simulators/