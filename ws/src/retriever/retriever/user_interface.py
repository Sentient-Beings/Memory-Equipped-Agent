import customtkinter as ctk
from PIL import Image
import os
import threading
from retriever_agent import RetrieverAgent

class RAGNavigationGUI:
    def __init__(self):
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("dark-blue")

        self.root = ctk.CTk()
        self.root.title("RAG-based Navigation Chat")
        self.root.geometry("500x700")

        self.chat_frame = ctk.CTkScrollableFrame(self.root)
        self.chat_frame.pack(fill="both", expand=True, padx=10, pady=10)

        self.input_frame = ctk.CTkFrame(self.root)
        self.input_frame.pack(fill="x", padx=10, pady=(0, 10))

        self.input_entry = ctk.CTkEntry(self.input_frame, placeholder_text="Type a message...", height=40, font=("Arial", 14))
        self.input_entry.pack(side="left", fill="x", expand=True, padx=(0, 10))

        self.send_button = ctk.CTkButton(self.input_frame, text="Send", command=self.send_message, width=80, height=40, font=("Arial", 14))
        self.send_button.pack(side="right")

        self.input_entry.bind("<Return>", lambda event: self.send_message())

        self.user_avatar = self.load_avatar("user_avatar.png")
        self.agent_avatar = self.load_avatar("robot_avatar.png")

        self.retriever_agent = RetrieverAgent()
        self.add_message("Agent", "Welcome to the demo for RAG-based navigation. How can I assist you?", self.agent_avatar)

    def load_avatar(self, filename):
        script_dir = os.path.dirname(os.path.realpath(__file__))
        image_path = os.path.join(script_dir, filename)
        if os.path.exists(image_path):
            return ctk.CTkImage(light_image=Image.open(image_path), size=(50, 50))
        else:
            print(f"Avatar image not found: {image_path}")
            return None

    def send_message(self):
        user_input = self.input_entry.get().strip()
        if user_input.lower() == 'exit':
            self.add_message("Agent", "Exiting the program. Goodbye!", self.agent_avatar)
            self.root.after(2000, self.root.quit)  # Close the window after 2 seconds
        elif user_input:
            self.add_message("User", user_input, self.user_avatar)
            self.input_entry.delete(0, ctk.END)
            threading.Thread(target=self.process_query, args=(user_input,)).start()
        else:
            self.add_message("Agent", "Empty input. Please try again.", self.agent_avatar)

    def process_query(self, user_input):
        self.add_message("Agent", "Processing your query...", self.agent_avatar)
        agent_response = self.retriever_agent.execute_graph(user_input)
        self.add_message("Agent", agent_response["context"], self.agent_avatar)

    def add_message(self, sender, message, avatar):
        frame = ctk.CTkFrame(self.chat_frame, fg_color="transparent")
        frame.pack(fill="x", padx=5, pady=5)

        if sender == "User":
            if avatar:
                avatar_label = ctk.CTkLabel(frame, image=avatar, text="")
                avatar_label.pack(side="right", padx=(5, 0))
            
            msg_frame = ctk.CTkFrame(frame, fg_color="#2C3E50")
            msg_frame.pack(side="right", padx=5, pady=5)
        else:
            if avatar:
                avatar_label = ctk.CTkLabel(frame, image=avatar, text="")
                avatar_label.pack(side="left", padx=(0, 5))
            
            msg_frame = ctk.CTkFrame(frame, fg_color="#34495E")
            msg_frame.pack(side="left", padx=5, pady=5)

        text = ctk.CTkLabel(msg_frame, text=message, wraplength=300, font=("Arial", 14))
        text.pack(padx=10, pady=5)

        self.chat_frame._parent_canvas.yview_moveto(1.0)

    def run(self):
        self.root.mainloop()

if __name__ == '__main__':
    app = RAGNavigationGUI()
    app.run()