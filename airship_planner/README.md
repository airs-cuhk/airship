# airship_planner
**This package implements a task planner leveraging LLMs. It supports both using OpenAI's API to access GPT modules and utilizing the Ollam API to access open-source LLMs, such as LLAMA.**

## airship_planner API Overview
### llm_planner_node
* Default configuration file: airship_planner/config/config.yaml
    * `semantic_map_file`: Specifies the file of the semantic file
    * `llm_server_url`: Specifies the url of the LLM service constructed by the user using Ollama.

1. Service
* `/airship_planner/planner_server`: Provides a service for receiving user instructions, utilizing LLMs to process those instructions, and generating a list of tasks composed of navigation, grasping, and placing. 

2. Parameters
* `semantic_map_dir`: The directory containing the semantic map. Users can specify this parameter through the launch file.
* `semantic_map_file`: The file name of the semantic map. Users can specify this parameter through config.yaml.
* `llm_server_url`: The URL of LLM server. User can specify this parameter through config.yaml.

## To-Do List
- [ ] Add more task primitives to the task planner and implement these new tasks in Airship. 
- [ ] Use VLM models to enhance the capbility of task planning and scheduling.
- [ ] Develop a foundation world model for general embodied AI applications.
