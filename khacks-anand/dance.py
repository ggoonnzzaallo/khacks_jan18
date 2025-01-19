from pydub import AudioSegment
from pydub.playback import play
import numpy as np

# Load the MP3 file
audio_file = "iron_man_slow.m4a"  # Replace with your file name
audio = AudioSegment.from_file(audio_file)

# Parameters
CHUNK_DURATION_MS = 100  # Chunk size in milliseconds
SAMPLE_RATE = audio.frame_rate

def process_audio(chunk):
    """Process audio chunk to extract amplitude and dominant frequency."""
    # Convert audio data to numpy array
    samples = np.array(chunk.get_array_of_samples())
    
    # Calculate amplitude
    amplitude = np.linalg.norm(samples)
    
    # Perform FFT to get frequency spectrum
    frequencies = np.fft.fft(samples)
    magnitudes = np.abs(frequencies[:len(frequencies)//2])  # Only positive frequencies
    
    # Get dominant frequency
    dominant_frequency = np.argmax(magnitudes) * SAMPLE_RATE / len(samples)
    
    return amplitude, dominant_frequency

# Break the audio into chunks and process
for i in range(0, len(audio), CHUNK_DURATION_MS):
    chunk = audio[i:i+CHUNK_DURATION_MS]
    amplitude, dominant_frequency = process_audio(chunk)
    
    # Print the processed data
    print({
        "time": f"{i/1000:.2f}s",  # Timestamp in seconds
        "amplitude": round(amplitude, 2),
        "frequency": round(dominant_frequency, 2)
    })
