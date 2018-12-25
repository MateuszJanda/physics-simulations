#!/bin/bash

if [[ $1 == "-p" || $1 == "--povs-to-pngs" ]]; then
    # Convert all *.pov files in current directory to .png files
    for i in *.pov; do povray Height=400 Width=400 Display=false Output_File_Type=N $i 2>/dev/null; echo "Converting: $i to png"; done
elif [[ $1 == "-g" || $1 == "--pngs-to-gif" ]]; then
    PALETTE_FILE="tmp_pallete.png"
    VIDEO_FILE="output_video.mp4"
    INPUT_FILES="img-%4d.png"
    OUTPUT_GIF="output.gif"
    FILTERS="fps=25"

    # Create video from *.png files
    ffmpeg -r 100 -i $INPUT_FILES -c:v libx264 -crf 0 -preset veryslow $VIDEO_FILE
    # Convert video file to .gif
    ffmpeg -v warning -i $VIDEO_FILE -vf "$FILTERS,palettegen" -y $PALETTE_FILE
    ffmpeg -v warning -i $VIDEO_FILE -i $PALETTE_FILE -lavfi "$FILTERS [x]; [x][1:v] paletteuse" -y $OUTPUT_GIF
else
    echo "Warning. No parameters."
    echo "Usage:"
    echo "   ./gif_maker.sh --povs-to-pngs"
    echo "   ./gif_maker.sh --pngs-to-gif"
fi
