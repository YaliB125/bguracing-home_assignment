import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import pandas as pd

class CsvSender(Node):
    def __init__(self):
        super().__init__('csv_sender_node')
        self.publisher_ = self.create_publisher(Point, 'cone_locations', 10)

        self.df = pd.read_csv('/ros2_ws/data/BrandsHatchLayout.csv', names=['x', 'y', 'side'], header=None)
        
        if isinstance(self.df.iloc[0]['x'], str):
             self.df = self.df.iloc[1:]
             
        self.df = self.df.dropna().reset_index(drop=True)
        
        self.df['x'] = self.df['x'].astype(float)
        self.df['y'] = self.df['y'].astype(float)

        self.data_iterator = self.df.iterrows()
        self.timer = self.create_timer(0.01, self.publish_point) 
    def publish_point(self):
        try:
            index, row = next(self.data_iterator)

            msg = Point()
            msg.x = float(row['x'])
            msg.y = float(row['y'])

            side = str(row['side']).lower()
            if 'left' in side:
                msg.z = 1.0
            elif 'right' in side:
                msg.z = 2.0
            
            self.publisher_.publish(msg)

        except StopIteration:
            self.data_iterator = self.df.iterrows()
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = CsvSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()