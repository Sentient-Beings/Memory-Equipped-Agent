from typing import Annotated, Sequence, TypedDict , Literal
from langchain_core.messages import SystemMessage, HumanMessage, AIMessage, ToolMessage , AnyMessage , BaseMessage , RemoveMessage
from langgraph.graph.message import add_messages
from langgraph.graph import StateGraph, START, END
from langchain_groq import ChatGroq
from langchain_core.tools import tool
from langgraph.prebuilt import ToolNode
from langgraph.prebuilt import tools_condition
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.runnables import RunnablePassthrough
from langchain_core.output_parsers import StrOutputParser
from langchain_community.embeddings import HuggingFaceBgeEmbeddings
from langchain_pinecone import PineconeVectorStore
from pinecone import Pinecone, ServerlessSpec
from dotenv import load_dotenv
import time
import os

load_dotenv()

GROQ_API_KEY        = os.getenv("GROQ_API_KEY")
HUGGINGFACE_API_KEY = os.getenv("HUGGINGFACE_API_KEY")
PINECONE_API_KEY    = os.getenv("PINECONE_API_KEY")
LANGCHAIN_API_KEY   = os.getenv("LANGCHAIN_API_KEY")


class RetrieverAgent:
    def __init__(self):
        self.model = ChatGroq(
            model="llama3-70b-8192",
            temperature=0,
            max_tokens=1000,
            request_timeout=60
        )
        self.pc = Pinecone(api_key=PINECONE_API_KEY)
        self.model_name = "BAAI/bge-m3"
        self.model_kwargs = {"device": "cuda"}
        self.encode_kwargs = {"normalize_embeddings": True}
        self.embeddings = HuggingFaceBgeEmbeddings(
            model_name=self.model_name, model_kwargs=self.model_kwargs, encode_kwargs=self.encode_kwargs)
        self.index_name = "rag-for-robots"
        self.existing_indexes = [index_info["name"] for index_info in self.pc.list_indexes()]
        if self.index_name not in self.existing_indexes:
            self.pc.create_index(
                name=self.index_name,
                dimension=1024,
                metric="cosine",
                spec=ServerlessSpec(cloud="aws", region="us-east-1"),
            )
            while not self.pc.describe_index(self.index_name).status["ready"]:
                time.sleep(1)

        index = self.pc.Index(self.index_name)
        self.vector_store = PineconeVectorStore(index=index, embedding=self.embeddings)
        self.set_template = """Extract the location coordinates according to the users query: 
                    {context}

                    Query: {query}
                """
        self.set_prompt = ChatPromptTemplate.from_template(self.set_template)
        
    class RetrieverState(TypedDict):
        user_query: str
        context: str
        messages: Annotated[list[AnyMessage], add_messages]
        nav_success: bool

        
    def retrieval_node(RetrieverState) -> RetrieverState:
        user_query = RetrieverState["user_query"]

        retriever = self.vector_store.as_retriever(
            search_type="similarity",
            search_kwargs={"k": 1},
        )

        context = retriever.invoke(f"{user_query}", filter={"source": "visited location"})
        return {"context": context}
        
        
    @tool
    def navigate_to_goal(
        x: Annotated[float, "Robot X location at goal"],
        y: Annotated[float, "Robot Y location at goal"],
        ) -> RetrieverState:
        '''
        Use this tool to send the goal coordinates (X,Y) for the robot to navigate to. This tool would ensure that robot moves to the 
        requested goal location.

        Args:
            x (float): X location of the goal
            y (float): Y location of the goal
        '''
        return {"nav_success": True}

    def tool_declaration(self):
        tools = [self.navigate_to_goal]
        return tools
    
    def reasoning_node(self, RetrieverState) -> RetrieverState:
        tools = self.tool_declaration()
        model_w_tools = self.model.bind_tools(tools)
        if RetrieverState["context"] is None:
            RetrieverState["context"] = "No Information available"
        chain = (
            {"context": lambda x:RetrieverState["context"], "query": RunnablePassthrough()}
            | self.set_prompt
            | model_w_tools
        )

        agent_response = chain.invoke(RetrieverState["user_query"])
        return {"messages": [agent_response]}
    
    def router(self, RetrieverState) -> Literal["navigate_to_goal", "__end__"]:
        messages = RetrieverState["messages"]
        last_message = messages[-1]
        if last_message.tool_calls:
            return "navigate_to_goal"
        return END
    
    def graph_definition(self):
        tools = self.tool_declaration()
        builder = StateGraph(self.RetrieverState)
        builder.add_node("retrieval", self.retrieval_node)
        builder.add_node("reasoning", self.reasoning_node)
        builder.add_node("navigate_to_goal", ToolNode(tools))

        builder.add_edge(START, "retrieval")
        builder.add_edge("retrieval", "reasoning")
        builder.add_conditional_edges(
            "reasoning",
            self.router,
        )
        builder.add_edge("navigate_to_goal", END)
        graph = builder.compile()
        return graph
    
    def execute_graph(self, user_query: str):
        initial_state = self.RetrieverState({
            "user_query": user_query,
            "context": None,
            "messages": [],
            "nav_success": False,   
            })
        graph = self.graph_definition()
        output = graph.invoke(initial_state)
        return output