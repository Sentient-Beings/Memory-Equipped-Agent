import plotly.graph_objects as go
import networkx as nx
from datetime import datetime
import colorsys
import json
import os

class SequentialSemanticMap:
    def __init__(self):
        self.graph = nx.Graph()
        self.last_node = None

    def add_location(self, x, y, description, timestamp):
        node_id = len(self.graph.nodes)
        self.graph.add_node(node_id, pos=(x, y), description=description, timestamp=timestamp)
        
        if self.last_node is not None:
            self.graph.add_edge(self.last_node, node_id)
        
        self.last_node = node_id
        return node_id

    def generate_bright_colors(self, num_colors):
        colors = []
        for i in range(num_colors):
            hue = i / num_colors
            saturation = 1.0
            value = 1.0
            rgb = colorsys.hsv_to_rgb(hue, saturation, value)
            colors.append(f'rgb({int(rgb[0]*255)}, {int(rgb[1]*255)}, {int(rgb[2]*255)})')
        return colors

    def visualize(self, output_path):
        pos = nx.get_node_attributes(self.graph, 'pos')
        
        edge_x = []
        edge_y = []
        for edge in self.graph.edges():
            x0, y0 = pos[edge[0]]
            x1, y1 = pos[edge[1]]
            edge_x.extend([x0, x1, None])
            edge_y.extend([y0, y1, None])

        edge_trace = go.Scatter(
            x=edge_x, y=edge_y,
            line=dict(width=1, color='rgba(255, 255, 255, 0.5)'),
            hoverinfo='none',
            mode='lines')

        node_x = [pos[node][0] for node in self.graph.nodes()]
        node_y = [pos[node][1] for node in self.graph.nodes()]

        bright_colors = self.generate_bright_colors(len(self.graph.nodes))

        node_trace = go.Scatter(
            x=node_x, y=node_y,
            mode='markers+text',
            hoverinfo='text',
            text=[f"{node}" for node in self.graph.nodes()],
            textposition="top center",
            textfont=dict(color='white'),
            marker=dict(
                showscale=False,
                size=15,
                color=bright_colors,
                line=dict(width=1, color='white')
            )
        )

        node_text = []
        for node, adjacencies in enumerate(self.graph.adjacency()):
            node_info = self.graph.nodes[node]
            node_text.append(f"Node: {node}<br>"
                             f"Location: ({node_info['pos'][0]}, {node_info['pos'][1]})<br>"
                             f"Description: {node_info['description']}<br>"
                             f"Timestamp: {node_info['timestamp']}")

        node_trace.hovertext = node_text

        fig = go.Figure(data=[edge_trace, node_trace],
                        layout=go.Layout( 
                            title={
                                'text': 'SEQUENTIAL SEMANTIC MAP',
                                'y': 0.95,
                                'x': 0.5,
                                'xanchor': 'center',
                                'yanchor': 'top',
                                'font': {
                                    'size': 24,
                                    'color': 'rgba(255, 165, 0, 0.7)',
                                    'family': 'Courier New, monospace'
                                }
                            },
                            showlegend=False,
                            hovermode='closest',
                            margin=dict(b=40, l=5, r=5, t=60),  
                            plot_bgcolor='rgb(17,17,17)',
                            paper_bgcolor='rgb(17,17,17)',
                            xaxis=dict(showgrid=False, zeroline=False, showticklabels=False),
                            yaxis=dict(showgrid=False, zeroline=False, showticklabels=False)
                        ))
        
        fig.update_xaxes(showgrid=True, gridwidth=1, gridcolor='rgba(255,255,255,0.1)')
        fig.update_yaxes(showgrid=True, gridwidth=1, gridcolor='rgba(255,255,255,0.1)')

        fig.add_annotation(
            text="SYSTEM ONLINE • MAPPING ACTIVE",
            xref="paper", yref="paper",
            x=0.01, y=0.01, 
            xanchor='left', yanchor='bottom',
            showarrow=False,
            font=dict(
                size=12,
                color="rgba(0, 255, 0, 0.7)",  
                family="Courier New, monospace"
            ),
            align="left",
        )
        
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        fig.write_html(output_path)
        print(f"Semantic map saved to: {output_path}")

if __name__ == "__main__":
    semantic_map = SequentialSemanticMap()
    
    with open('memory_chunks.jsonl', 'r') as file:
        memory_chunks = [json.loads(line) for line in file]

    for chunk in memory_chunks:
        x = chunk['Robot x coordinate']
        y = chunk['Robot y coordinate']
        caption = chunk['caption']
        timestamp = chunk['Timestamp']
        semantic_map.add_location(x, y, caption, timestamp)
    semantic_map.visualize("semantic_map/sequential_semantic_map.html")
