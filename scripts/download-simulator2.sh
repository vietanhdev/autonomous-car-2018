echo "Downloading Simulator 2"
mkdir -p tmp
cd tmp
../scripts/download-gdrive.sh "https://drive.google.com/uc?export=download&id=1gaL0vnchyOtrnE9rvKw0IQndWS-tG4_T" "simulator2.zip"
cd ..

echo "Extracing ZIP file"
mkdir -p simulators
unzip tmp/simulator2.zip -d simulators/