using System.Collections.Generic;
using System.Windows;

namespace ShearCell_Data.Model
{
    public class MetamaterialModel
    {
        public List<Vertex> Vertices { get; }
        public List<Edge> Edges { get; }
        public List<Cell> Cells { get; }

        public Dictionary<Vertex, List<Vector>> InputMotion { get; }
        public Dictionary<Vertex, List<Vector>> OutputMotion { get; }

        public List<List<Edge>> ConstraintsGraph { get; private set; }
        public List<List<List<Edge>>> ConnectedComponents { get; private set; }


        //public Vertex InputVertex { get; set; }
        //public List<Vector> InputPath { get; set; } 
        //public Vertex OutputVertex { get; set; }
        //public List<Vector> OutputPath { get; set; } 

        public MetamaterialModel()
        {
            Vertices = new List<Vertex>();
            Edges = new List<Edge>();
            Cells = new List<Cell>();

            InputMotion = new Dictionary<Vertex, List<Vector>>();
            OutputMotion = new Dictionary<Vertex, List<Vector>>();

            //InputPath = new List<Vector>();
            //OutputPath = new List<Vector>();
        }

        public void Clear()
        {
            Reset();

            Cells.Clear();
            Vertices.Clear();
            Edges.Clear();
        }

        public void Reset()
        {
            ResetDeformation();
            ResetInput();
        }

        public void ResetDeformation()
        {
            foreach (var vertex in Vertices)
                vertex.ResetDeformation();
        }

        public void ResetInput()
        {
            InputMotion.Clear();
            OutputMotion.Clear();

            //InputPath.Clear();
            //InputVertex = null;

            //OutputPath.Clear();
            //OutputVertex = null;
        }

        public List<Vertex> GetAnchors()
        {
            return Vertices.FindAll(cell => cell.IsAnchor);
        } 

        public void AddCell(Cell cell, Vector indexPosition, Size size)
        {
            var existingCell = Cells.Find(c => c.IndexVertex.Equals(indexPosition));
            if (existingCell != null)
                DeleteCell(indexPosition);

            var vertices = ContstructNewCellVertices(indexPosition, size);
            AddCellFromVertices(cell, vertices);
        }

        public void AddCellFromVertices(Cell cell, List<Vertex> vertices)
        {
            AddVertices(vertices, cell);
            AddEdges(vertices, cell);
            Cells.Add(cell);
            //LinkAdjacentVertices(vertices);
        }

        internal void DeleteCell(Vector indexPosition)
        {
            var cellToDelete = Cells.Find(cell => cell.IndexVertex.Equals(indexPosition));
            if(cellToDelete == null)
                return;

            foreach (var cellVertex in cellToDelete.CellVertices)
            {
                if (cellVertex.ContainingCells.Count <= 1)
                {
                    Vertices.Remove(cellVertex);
                    Edges.RemoveAll(edge => edge.Contains(cellVertex));
                }

                cellVertex.ContainingCells.Remove(cellToDelete);
            }

            Cells.Remove(cellToDelete);
        }

        public void AddAnchor(Vector gridPosition)
        {
            //var vertex = new Vertex(CoordinateConverter.ConvertScreenToGlobalCellCoordinates(ellipseScreenPosition));
            var vertex = new Vertex(gridPosition);
            var existingVertex = Vertices.Find(x => x.Equals(vertex));

            if (existingVertex == null)
                return;

            existingVertex.IsAnchor = !existingVertex.IsAnchor;

            //if(existingVertex.IsAnchor)
                PropagateAnchors(existingVertex);
        }

        public int GetDegreesOfFreedom()
        {
            BuildConstraintsGraph();
            GetConnectedComponents();

            return ConnectedComponents.Count;
        }

        private void PropagateAnchors(Vertex anchor)
        {
            var newAnchors = new List<Vertex>();

            foreach (var cell in anchor.ContainingCells)
            {
                if (cell.CanMove())
                    continue;

                foreach (var vertex in cell.CellVertices)
                {
                    if(vertex.IsAnchor)
                        continue;

                    vertex.IsAnchor = true;
                    newAnchors.Add(vertex);
                }
            }

            PropagateAnchors(newAnchors);
        }

        private void PropagateAnchors(List<Vertex> newAnchors)
        {
            foreach (var anchor in newAnchors)
            {
                foreach (var cell in anchor.ContainingCells)
                {
                    if (cell.CanMove())
                        continue;

                    foreach (var vertex in cell.CellVertices)
                    {
                        if (vertex.IsAnchor)
                            continue;

                        vertex.IsAnchor = true;
                    }
                }
            }
        }

        private List<Vertex> ContstructNewCellVertices(Vector indexPosition, Size size)
        {
            var vertices = new List<Vertex>
            {
                new Vertex(indexPosition), //bottom left 
                new Vertex(new Vector(indexPosition.X + size.Width, indexPosition.Y)), //bottom right
                new Vertex(new Vector(indexPosition.X + size.Width, indexPosition.Y + size.Height)), //top right
                new Vertex(new Vector(indexPosition.X, indexPosition.Y + size.Height)) //top left
            };

            return vertices;
        }

        private void AddVertices(List<Vertex> vertices, Cell cell)
        {
            for (var i = 0; i < vertices.Count; i++)
            {
                var vertex = vertices[i];

                if (!Vertices.Contains(vertex))
                    AddNewVertex(vertex, cell, i == 0);
                else
                    LinkExistingVertex(vertex, cell, i == 0);
            }
        }

        private void AddEdges(List<Vertex> vertices, Cell cell)
        {
            Edge edge;
            for (var i = 1; i < vertices.Count; i++)
            {
                edge = new Edge(vertices[i - 1], vertices[i]);

                if (Edges.Contains(edge))
                {
                    var existingEdge = Edges.Find(e => e.Equals(edge));
                    cell.CellEdges.Add(existingEdge);
                }
                else
                {
                    cell.CellEdges.Add(edge);
                    Edges.Add(edge);
                }
            }

            edge = new Edge(vertices[vertices.Count - 1], vertices[0]);
            if (Edges.Contains(edge))
            {
                var existingEdge = Edges.Find(e => e.Equals(edge));
                cell.CellEdges.Add(existingEdge);
            }
            else
            {
                cell.CellEdges.Add(edge);
                Edges.Add(edge);
            }
        }

        private void AddNewVertex(Vertex vertex, Cell cell, bool isFirst)
        {
            vertex.ContainingCells.Add(cell);

            if (isFirst)
                cell.IndexVertex = vertex;

            cell.CellVertices.Add(vertex);
            Vertices.Add(vertex);
        }

        private void LinkExistingVertex(Vertex vertex, Cell cell, bool isFirst)
        {
            var existingVertex = Vertices.Find(x => x.Equals(vertex));
            existingVertex.ContainingCells.Add(cell);

            if (isFirst)
                cell.IndexVertex = existingVertex;

            cell.CellVertices.Add(existingVertex);
        }

        private void BuildConstraintsGraph()
        {
            ConstraintsGraph = new List<List<Edge>>();

            foreach (var cell in Cells)
            {
                ConstraintsGraph.AddRange(cell.GetConstraintGraphRepresentation());

                //if (cell is RigidCell)
                //{
                //    ConstraintsGraph.Add(cell.CellEdges);
                //}
                //else
                //{
                //    //horizontal
                //    ConstraintsGraph.Add(new List<Edge> { cell.CellEdges[1], cell.CellEdges[3] });

                //    //vertical
                //    ConstraintsGraph.Add(new List<Edge> { cell.CellEdges[0], cell.CellEdges[2] });
                //}
            }
        }

        private void GetConnectedComponents()
        {
            ConnectedComponents = new List<List<List<Edge>>>();

            var toVisit = new List<List<Edge>>(ConstraintsGraph);

            var edgesToVisit = new List<Edge>();
            var visitedEdges = new List<Edge>();

            while (toVisit.Count > 0)
            {
                var subcomponent = toVisit[0];
                toVisit.RemoveAt(0);

                var component = new List<List<Edge>> {subcomponent};
                edgesToVisit.AddRange(subcomponent);

                while (edgesToVisit.Count > 0)
                {
                    var edge = edgesToVisit[0];
                    edgesToVisit.RemoveAt(0);

                    if (visitedEdges.Contains(edge))
                        continue;

                    visitedEdges.Add(edge);

                    var componentIndex = FindConnectedComponent(edge, toVisit);
                    if (componentIndex == -1)
                        continue;

                    var containingComponent = toVisit[componentIndex];
                    component.Add(containingComponent);
                    toVisit.RemoveAt(componentIndex);

                    edgesToVisit.AddRange(containingComponent);
                }

                ConnectedComponents.Add(component);
            }
        }

        private int FindConnectedComponent(Edge edge, List<List<Edge>> components)
        {
            foreach (var component in components)
            {
                if (component.Contains(edge))
                    return components.IndexOf(component);
            }

            return -1;
        }

        //private void LinkAdjacentVertices(List<Vertex> vertices)
        //{
        //    for (var i = 0; i < vertices.Count; i++)
        //    {
        //        var currentVertex = Vertices.Find(x => x.Equals(vertices[i]));
        //        var previousVertex = Vertices.Find(x => x.Equals(vertices[(i - 1 + vertices.Count) % vertices.Count]));
        //        var nextVertex = Vertices.Find(x => x.Equals(vertices[(i + 1 + vertices.Count) % vertices.Count]));

        //        if (!currentVertex.AdjacentVertices.Contains(previousVertex))
        //            currentVertex.AdjacentVertices.Add(previousVertex);
        //        if (!currentVertex.AdjacentVertices.Contains(nextVertex))
        //            currentVertex.AdjacentVertices.Add(nextVertex);
        //    }
        //}

    }
}
