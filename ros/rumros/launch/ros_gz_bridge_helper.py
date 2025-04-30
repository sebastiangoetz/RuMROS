import os
import tempfile
import sys

def replace_namespace_in_yaml(file_path: str, replacement: str) -> str:
    """
    Reads a YAML file, replaces occurrences of {{namespace}} with a given replacement,
    writes the modified YAML to a temporary file and returns the path to the temporary file.
    
    :param file_path: Path to the original YAML file
    :param replacement: String to replace {{namespace}} with
    :return: Path to the temporary YAML file with replacements
    """
    # Validate the file path
    if not os.path.isfile(file_path):
        raise FileNotFoundError(f"The file '{file_path}' does not exist.")
    
    # Read the content of the YAML file
    with open(file_path, 'r') as file:
        yaml_content = file.read()
    
    # Replace {{namespace}} with the given replacement string
    updated_content = yaml_content.replace('{{namespace}}', replacement)
    
    # Create a temporary file to save the updated YAML content
    temp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".yaml", mode='w')
    temp_file.write(updated_content)
    temp_file.close()
    
    # Return the path to the temporary file
    return temp_file.name

def main(argv):
    print("config_file:=" + replace_namespace_in_yaml(argv[0], argv[1]))

if __name__ == "__main__":
   main(sys.argv[1:])
