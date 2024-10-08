from retriever_agent import RetrieverAgent

def main():
    retriever_agent = RetrieverAgent()
    print("Welcome to the demo for RAG-based navigation. Type 'exit' to quit.")

    while True:
        user_input = input("Enter your query: ").strip()
        
        if user_input.lower() == 'exit':
            print("Exiting the program. Goodbye!")
            break

        if user_input:
            print("Processing your query...")
            agent_response = retriever_agent.execute_graph(user_input)
            print(f"Agent response: {agent_response}")
        else:
            print("Empty input. Please try again.")

if __name__ == '__main__':
    main()