import os
import sys
import struct

def wav_to_header(wav_file, output_header="SoundData.h", track_id=0):
    """
    Reads a WAV file and converts its data into a C header file with a byte array.
    Supports appending multiple tracks to the same header file.
    """
    if not os.path.exists(wav_file):
        print(f"Error: File '{wav_file}' not found.")
        return

    try:
        with open(wav_file, "rb") as f:
            # Read WAV header
            header = f.read(44)
            
            # Simple validation (RIFF, WAVE)
            if header[0:4] != b'RIFF' or header[8:12] != b'WAVE':
                print("Error: Not a valid WAV file.")
                return

            # Parse format details
            channels = struct.unpack('<H', header[22:24])[0]
            sample_rate = struct.unpack('<I', header[24:28])[0]
            bits_per_sample = struct.unpack('<H', header[34:36])[0]
            data_size = struct.unpack('<I', header[40:44])[0]

            print(f"Converting '{wav_file}' (ID: {track_id}):")
            print(f"  - Sample Rate: {sample_rate} Hz")
            print(f"  - Channels: {channels}")
            print(f"  - Bits: {bits_per_sample}-bit")
            print(f"  - Data Size: {data_size} bytes")

            # Warning for non-standard formats (ESP32 I2S config dependent)
            if sample_rate != 44100 and sample_rate != 11025 and sample_rate != 4800:
                print(f"Warning: Sample rate is {sample_rate}Hz (not 44100Hz, 11025Hz or 4800Hz). Audio may play at wrong speed.")
            if bits_per_sample != 16:
                print("Warning: Not 16-bit audio. Audio noise/distortion may occur.")

            # Read audio data
            audio_data = f.read()
            
    except Exception as e:
        print(f"Error reading WAV file: {e}")
        return

    # Define variable names
    array_name = f"sound_data_{track_id}"
    len_name = f"sound_data_len_{track_id}"

    # Prepare the C array content
    c_array_content = f"""
// Source File: {os.path.basename(wav_file)}
// Format: {sample_rate}Hz, {bits_per_sample}-bit, {channels} ch
// Length: {len(audio_data)} bytes
const unsigned int {len_name} = {len(audio_data)};
const unsigned char {array_name}[] = {{
"""
    
    # Format hex data
    hex_lines = []
    line_buffer = []
    for i, byte in enumerate(audio_data):
        line_buffer.append(f"0x{byte:02X}")
        if len(line_buffer) == 12:
            hex_lines.append("  " + ", ".join(line_buffer) + ", ")
            line_buffer = []
    
    if line_buffer:
        hex_lines.append("  " + ", ".join(line_buffer) + ", ")
        
    c_array_content += "\n".join(hex_lines)
    c_array_content += "\n};\n"

    # Write to file
    mode = "r+" if os.path.exists(output_header) else "w"
    
    try:
        if mode == "w":
            # Create new file with header guards
            with open(output_header, "w") as out:
                out.write(f"#ifndef SOUND_DATA_H\n#define SOUND_DATA_H\n\n{c_array_content}\n#endif\n")
            print(f"Created '{output_header}' with track ID {track_id}.")
        else:
            # Append to existing file (insert before #endif)
            with open(output_header, "r") as f:
                content = f.read()
            
            # Check if variable already exists to avoid duplicates
            if f"const unsigned char {array_name}[]" in content:
                print(f"Warning: Track ID {track_id} already exists in {output_header}. Overwriting file is safer manually.")
                # Basic overwrite logic could be complex, so we just append and warn
                # ideally we should remove the old one, but simple append is requested.
                # Let's just insert before #endif
            
            if "#endif" in content:
                new_content = content.replace("#endif", f"{c_array_content}\n#endif")
            else:
                new_content = content + f"\n{c_array_content}\n#endif\n"
            
            with open(output_header, "w") as out:
                out.write(new_content)
            print(f"Appended track ID {track_id} to '{output_header}'.")
            
    except Exception as e:
        print(f"Error writing header file: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python wav_to_header.py <input.wav> [output_header_path] [track_id]")
        print("Example: python wav_to_header.py coin.wav SoundData.h 0")
        print("Example (Append): python wav_to_header.py bgm.wav SoundData.h 1")
    else:
        input_wav = sys.argv[1]
        output_header = sys.argv[2] if len(sys.argv) > 2 else "SoundData.h"
        track_id = sys.argv[3] if len(sys.argv) > 3 else 0
        wav_to_header(input_wav, output_header, track_id)
