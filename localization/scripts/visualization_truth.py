import rosbag
import csv

# Replace with your actual bag file path
bag_path = "/home/ramzy/roar_ws/direct.bag"

# Define the topic name and desired fields to extract
topic_name = "/ground_truth"
fields = ["data"]  # Adjust field names as needed

# Open the ROS bag
bag = rosbag.Bag(bag_path)

# Open a CSV file for writing
with open("extracted_data_truth.csv", 'w', newline='') as csvfile:
  # Create a CSV writer object
  csv_writer = csv.writer(csvfile)
  
  # Write header row with field names
  csv_writer.writerow(fields)

  # Iterate over messages in the bag
  for topic, msg, t in bag.read_messages(topics=[topic_name]):
    # Check if the message is from the desired topic
    if topic == topic_name:
      # Extract data from desired fields
      extracted_data = {field: getattr(msg, field) for field in fields}
      # Add timestamp (convert rospy.Time to milliseconds)
    #   extracted_data["timestamp"] = t.to_nsec() / 1e6  # Convert to milliseconds
      
      # Write extracted data as a row to CSV file
      csv_writer.writerow(extracted_data.values())

# Close the ROS bag
bag.close()

print("Extracted data saved to extracted_data.csv")