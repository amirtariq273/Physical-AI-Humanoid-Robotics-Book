import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData # Assuming ros2_audio_common
import whisper
import numpy as np
import threading
import collections

class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')
        self.declare_parameter('whisper_model', 'base')
        self.declare_parameter('audio_topic', '/audio')
        self.declare_parameter('transcription_topic', '/voice_commands')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('audio_buffer_duration', 5) # seconds

        self.whisper_model_name = self.get_parameter('whisper_model').get_parameter_value().string_value
        self.audio_topic = self.get_parameter('audio_topic').get_parameter_value().string_value
        self.transcription_topic = self.get_parameter('transcription_topic').get_parameter_value().string_value
        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        self.audio_buffer_duration = self.get_parameter('audio_buffer_duration').get_parameter_value().integer_value

        self.get_logger().info(f"Loading Whisper model: {self.whisper_model_name}")
        self.model = whisper.load_model(self.whisper_model_name)
        self.get_logger().info("Whisper model loaded.")

        self.audio_subscription = self.create_subscription(
            AudioData,
            self.audio_topic,
            self.audio_callback,
            10)
        self.transcription_publisher = self.create_publisher(
            String,
            self.transcription_topic,
            10)

        self.audio_buffer = collections.deque()
        self.buffer_lock = threading.Lock()
        self.transcription_thread = threading.Thread(target=self.transcribe_loop)
        self.transcription_thread.daemon = True
        self.transcription_thread.start()
        
        self.get_logger().info(f"Whisper node started, subscribing to {self.audio_topic}")

    def audio_callback(self, msg):
        # Convert bytes to numpy array (assuming float32 or int16, adjust as needed)
        # Here we assume int16 for common audio sources, and convert to float32 for Whisper
        audio_np = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0
        
        with self.buffer_lock:
            self.audio_buffer.extend(audio_np)
            # Trim buffer to maintain a fixed duration
            max_samples = self.sample_rate * self.audio_buffer_duration
            while len(self.audio_buffer) > max_samples:
                self.audio_buffer.popleft()

    def transcribe_loop(self):
        while rclpy.ok():
            audio_segment = None
            with self.buffer_lock:
                if len(self.audio_buffer) > self.sample_rate * 1.0: # Only transcribe if we have at least 1 second of audio
                    audio_segment = np.array(self.audio_buffer, dtype=np.float32)
                    self.audio_buffer.clear() # Clear buffer after processing
            
            if audio_segment is not None:
                self.get_logger().info(f"Transcribing {len(audio_segment)/self.sample_rate:.2f} seconds of audio...")
                try:
                    # Whisper expects 16kHz mono audio
                    result = self.model.transcribe(audio_segment, fp16=False) # fp16=False if no GPU
                    text = result["text"].strip()
                    if text:
                        self.get_logger().info(f"Transcription: '{text}'")
                        msg = String()
                        msg.data = text
                        self.transcription_publisher.publish(msg)
                except Exception as e:
                    self.get_logger().error(f"Error during transcription: {e}")
            rclpy.spin_once(self, timeout_sec=0.1) # Process ROS events while waiting for audio


def main(args=None):
    rclpy.init(args=args)
    whisper_node = WhisperNode()
    rclpy.spin(whisper_node)
    whisper_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()