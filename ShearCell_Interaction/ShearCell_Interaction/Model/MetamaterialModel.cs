using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Windows;
using ShearCell_Interaction.Helper;
using ShearCell_Interaction.Properties;
using ShearCell_Interaction.Simulation;

namespace ShearCell_Interaction.Model
{
    public class MetamaterialModel
    {
        public List<Vertex> Vertices { get; private set; }
        public List<Edge> Edges { get; private set; }
        public List<Cell> Cells { get; private set; }

        public Vertex InputVertex { get; set; }
        public List<Vector> InputPath { get; set; }
        public Vertex OutputVertex { get; set; }
        public List<Vector> OutputPath { get; set; }

        public List<HashSet<HashSet<Edge>>> ConstraintGraph { get; private set; }
        public int CurrentDoF
        {
            get { return ConstraintGraph.Count; }
        }

        public MetamaterialModel()
        {
            Vertices = new List<Vertex>();
            Edges = new List<Edge>();
            Cells = new List<Cell>();

            InputPath = new List<Vector>();
            OutputPath = new List<Vector>();


            ConstraintGraph = new List<HashSet<HashSet<Edge>>>();
        }

        public MetamaterialModel(MetamaterialModel other) : this()
        {
            foreach (var cell in other.Cells)
            {
                if (cell is ShearCell)
                {
                    AddCell(new ShearCell(), cell.IndexVertex.ToVector(), new Size(1,1), false);
                }
                else
                {
                    AddCell(new RigidCell(), cell.IndexVertex.ToVector(), new Size(1, 1), false);
                }
            }

            var anchors = other.GetAnchors();

            foreach (var anchor in anchors)
            {
                SetAnchor(anchor.ToVector(), true);
            }
            
            InputPath = new List<Vector>(other.InputPath.Select(v => new Vector(v.X, v.Y)));
            OutputPath = new List<Vector>(other.OutputPath.Select(v => new Vector(v.X, v.Y)));

            InputVertex = new Vertex(other.InputVertex.ToVector());
            OutputVertex = new Vertex(other.OutputVertex.ToVector());
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
            InputPath.Clear();
            InputVertex = null;

            OutputPath.Clear();
            OutputVertex = null;
        }

        public List<Vertex> GetAnchors()
        {
            return Vertices.FindAll(cell => cell.IsAnchor);
        }

        public Cell TryGetExistingCell(Vector indexPosition)
        {
            return Cells.Find(c => c.IndexVertex.Equals(indexPosition));
        }

        public void AddCell(Cell cell, Vector indexPosition, Size size, bool updateConstraintGraph = true)
        {
            var existingCell = Cells.Find(c => c.IndexVertex.Equals(indexPosition));
            if (existingCell != null)
                DeleteCell(indexPosition, updateConstraintGraph);

            var vertices = ContstructNewCellVertices(indexPosition, size);
            AddCellFromVertices(cell, vertices);

            //AddCellToGraph(cell);
            if (updateConstraintGraph)
                UpdateConstraintsGraph();
        }

        public void AddCellFromVertices(Cell cell, List<Vertex> vertices)
        {
            var usedVertices = AddVertices(vertices, cell);
            AddEdges(usedVertices, cell);
            cell.SetConstraints();
            Cells.Add(cell);
        }

        internal void DeleteCell(Vector indexPosition, bool updateConstraintGraph = true)
        {
            var cellToDelete = Cells.Find(cell => cell.IndexVertex.Equals(indexPosition));
            if (cellToDelete == null)
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

            foreach (var cellEdge in cellToDelete.CellEdges)
                cellEdge.ContainingCells.Remove(cellToDelete);

            Cells.Remove(cellToDelete);

            //RemoveCellFromGraph(cellToDelete);
            if (updateConstraintGraph)
                UpdateConstraintsGraph();
        }
        public void SetAnchor(Vector gridPosition, bool enabled)
        {
            var vertex = new Vertex(gridPosition);
            var existingVertex = Vertices.Find(x => x.Equals(vertex));

            if (existingVertex == null)
                return;

            existingVertex.IsAnchor = enabled;

            //if(existingVertex.IsAnchor)
            //PropagateAnchors(existingVertex);
        }

        public void ToggleAnchor(Vector gridPosition)
        {
            var vertex = new Vertex(gridPosition);
            var existingVertex = Vertices.Find(x => x.Equals(vertex));

            if (existingVertex == null)
                return;

            existingVertex.IsAnchor = !existingVertex.IsAnchor;

            //if(existingVertex.IsAnchor)
            //PropagateAnchors(existingVertex);
        }

        public void AddAnchor(Vector gridPosition)
        {
            var vertex = new Vertex(gridPosition);
            var existingVertex = Vertices.Find(x => x.Equals(vertex));

            if (existingVertex == null)
                return;

            existingVertex.IsAnchor = true;

            //if(existingVertex.IsAnchor)
            //PropagateAnchors(existingVertex);
        }

        public void PropagateAnchors(Vertex anchor)
        {
            var newAnchors = new List<Vertex>();

            foreach (var cell in anchor.ContainingCells)
            {
                if (cell.CanMove())
                    continue;

                foreach (var vertex in cell.CellVertices)
                {
                    if (vertex.IsAnchor)
                        continue;

                    vertex.IsAnchor = true;
                    newAnchors.Add(vertex);
                }
            }

            PropagateAnchors(newAnchors);
        }

        public void MoveToCoordinateOrigin()
        {
            var minVertex = Vertices.Min().ToVector();

            if (MathHelper.IsEqualDouble(minVertex.Length, 0))
                return;

            foreach (var vertex in Vertices)
            {
                var currentPosition = vertex.ToInitialVector();
                var newPosition = Vector.Subtract(currentPosition, minVertex);
                vertex.MoveTo(newPosition);
            }

            InputPath = MovePathBy(minVertex, InputPath);
            OutputPath = MovePathBy(minVertex, OutputPath);
        }

        public void Scale(int scaleFactor, bool scalePaths = false)
        {
            var tempCells = new List<Cell>();

            foreach (var cell in Cells)
            {
                tempCells.Add(cell);
            }

            var anchors = GetAnchors();

            ResetDeformation();

            Cells.Clear();
            Vertices.Clear();
            Edges.Clear();

            foreach (var tempCell in tempCells)
            {
                for (int x = 0; x < scaleFactor; x++)
                {
                    for (int y = 0; y < scaleFactor; y++)
                    {
                        int indexX = (int)tempCell.IndexVertex.X * scaleFactor + x;
                        int indexY = (int)tempCell.IndexVertex.Y * scaleFactor + y;

                        var indexPosition = new Vector(indexX, indexY);

                        if (tempCell is ShearCell)
                            AddCell(new ShearCell(), indexPosition, new Size(1, 1), false);
                        else
                            AddCell(new RigidCell(), indexPosition, new Size(1, 1), false);
                    }
                }
            }
            
            foreach(var anchor in anchors)
            {
                var scaledAnchorVector = anchor.ToVector() * scaleFactor;
                SetAnchor(scaledAnchorVector, true);
            }                        

            UpdateConstraintsGraph();

            if (scalePaths)
            {
                ScalePath(scaleFactor);
            }
            else
            {
                MovePaths(scaleFactor);
            }
        }

        internal void DecreaseScale(bool scalePaths = false)
        {
            var tempCells = new List<Cell>();

            foreach (var cell in Cells)
            {
                tempCells.Add(cell);
            }

            var anchors = GetAnchors();

            ResetDeformation();

            Cells.Clear();
            Vertices.Clear();
            Edges.Clear();

            Dictionary<Vector, List<Cell>> cellsPerScaledIndex = new Dictionary<Vector, List<Cell>>();

            foreach (var tempCell in tempCells)
            {
                int indexX = (int)Math.Floor(tempCell.IndexVertex.X * .5);
                int indexY = (int)Math.Floor(tempCell.IndexVertex.Y * .5);

                var indexPosition = new Vector(indexX, indexY);

                if (cellsPerScaledIndex.ContainsKey(indexPosition))
                {
                    cellsPerScaledIndex[indexPosition].Add(tempCell);
                }
                else
                {
                    cellsPerScaledIndex.Add(indexPosition, new List<Cell> { tempCell });
                }
            }

            foreach (var indexPosition in cellsPerScaledIndex.Keys)
            {
                var currentCells = cellsPerScaledIndex[indexPosition];
                var numRigid = currentCells.Count(cell => cell is RigidCell);
                var numShearCells = currentCells.Count(cell => cell is ShearCell);
                var numVoidCells = 4 - currentCells.Count; //each new scaled cell is composed of max 4 cells. if count is less, then some cells were empty

                //Console.WriteLine(numRigid + " " + numShearCells + " " + numVoidCells);

                if (numRigid > numShearCells && numRigid > numVoidCells)
                {
                    AddCell(new RigidCell(), indexPosition, new Size(1, 1), false);
                }
                else if (numShearCells > numRigid && numShearCells > numVoidCells)
                {
                    AddCell(new ShearCell(), indexPosition, new Size(1, 1), false);
                }
                else
                {
                    AddCell(new ShearCell(), indexPosition, new Size(1, 1), false);
                }
            }

            foreach (var anchor in anchors)
            {
                var scaledAnchorVector = anchor.ToVector() * .5;
                SetAnchor(scaledAnchorVector, true);
            }

            UpdateConstraintsGraph();
            
            if (scalePaths)
            {
                ScalePath(.5);
            }
            else
            {
                MovePaths(.5);
            }
        }
        

        private void ScalePath(double scaleFactor)
        {
            if (InputVertex != null && InputPath != null && InputPath.Count > 0)
            {
                var inputVector = InputVertex.ToVector();
                List<Vector> scaledOffsets = GetScaledOffsets(inputVector, InputPath, scaleFactor);

                InputVertex = new Vertex(inputVector * scaleFactor);
                var scaleInputVector = InputVertex.ToVector();
                for (int i = 0; i < InputPath.Count; i++)
                {
                    InputPath[i] = scaleInputVector + scaledOffsets[i];
                }

            }
            if (OutputVertex != null && OutputPath != null && OutputPath.Count > 0)
            {
                var outputVector = OutputVertex.ToVector();
                List<Vector> scaledOffsets = GetScaledOffsets(outputVector, OutputPath, scaleFactor);

                OutputVertex = new Vertex(outputVector * scaleFactor);
                var scaleOutputVector = OutputVertex.ToVector();
                for (int i = 0; i < OutputPath.Count; i++)
                {
                    OutputPath[i] = scaleOutputVector + scaledOffsets[i];
                }
            }
        }

        private static List<Vector> GetScaledOffsets(Vector start, List<Vector> points, double scaleFactor)
        {
            var scaledOffsets = new List<Vector>();

            foreach (var point in points)
            {
                var offset = point - start;
                var scaledOffset = offset * scaleFactor;
                scaledOffsets.Add(scaledOffset);
            }

            return scaledOffsets;
        }

        private void MovePaths(double scaleFactor)
        {
            if (InputVertex != null && InputPath != null && InputPath.Count > 0)
            {
                var inputVector = InputVertex.ToVector();
                List<Vector> unscaledOffsets = GetScaledOffsets(inputVector, InputPath, 1.0);

                InputVertex = new Vertex(inputVector * scaleFactor);
                var scaleInputVector = InputVertex.ToVector();
                for (int i = 0; i < InputPath.Count; i++)
                {
                    InputPath[i] = scaleInputVector + unscaledOffsets[i];
                }

            }
            if (OutputVertex != null && OutputPath != null && OutputPath.Count > 0)
            {
                var outputVector = OutputVertex.ToVector();
                List<Vector> unscaledOffsets = GetScaledOffsets(outputVector, OutputPath, 1.0);

                OutputVertex = new Vertex(outputVector * scaleFactor);
                var scaleOutputVector = OutputVertex.ToVector();
                for (int i = 0; i < OutputPath.Count; i++)
                {
                    OutputPath[i] = scaleOutputVector + unscaledOffsets[i];
                }
            }
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

        private List<Vertex> AddVertices(List<Vertex> vertices, Cell cell)
        {
            var usedVertices = new List<Vertex>();
            for (var i = 0; i < vertices.Count; i++)
            {
                var vertex = vertices[i];

                if (!Vertices.Contains(vertex))
                {
                    AddNewVertex(vertex, cell, i == 0);
                    usedVertices.Add(vertex);
                }
                else
                {
                    var existingVertex = LinkExistingVertex(vertex, cell, i == 0);
                    usedVertices.Add(existingVertex);
                }
            }

            return usedVertices;
        }

        private void AddEdges(List<Vertex> vertices, Cell cell)
        {
            Edge edge;
            for (var i = 1; i < vertices.Count; i++)
            {
                edge = new Edge(vertices[i - 1], vertices[i]);
                AddEdge(edge, cell);
            }

            edge = new Edge(vertices[vertices.Count - 1], vertices[0]);
            AddEdge(edge, cell);
        }

        private void AddEdge(Edge edge, Cell cell)
        {
            if (Edges.Contains(edge))
            {
                var existingEdge = Edges.Find(e => e.Equals(edge));
                cell.CellEdges.Add(existingEdge);
                existingEdge.ContainingCells.Add(cell);
            }
            else
            {
                cell.CellEdges.Add(edge);
                edge.ContainingCells.Add(cell);
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

        private Vertex LinkExistingVertex(Vertex vertex, Cell cell, bool isFirst)
        {
            var existingVertex = Vertices.Find(x => x.Equals(vertex));
            existingVertex.ContainingCells.Add(cell);

            if (isFirst)
                cell.IndexVertex = existingVertex;

            cell.CellVertices.Add(existingVertex);
            return existingVertex;
        }

        private List<Vector> MovePathBy(Vector offset, List<Vector> path)
        {
            if (path == null || path.Count <= 0)
                return path;

            var movedPath = new List<Vector>();

            foreach (var vector in path)
                movedPath.Add(Vector.Subtract(vector, offset));

            return movedPath;
        }

        private void AddToGraph(Edge key, List<Edge> values, Dictionary<Edge, List<Edge>> graph)
        {
            if (!graph.ContainsKey(key))
                graph.Add(key, values);
            else
                graph[key].AddRange(values);
        }

        private void AddCellToGraph(Cell cell)
        {
            hasConstraintGraphChanged = true;
            Debug.WriteLine("(AddCellToGraph) hasConstraintGraphChanged = " + hasConstraintGraphChanged);


            var stopWatch = new Stopwatch();
            stopWatch.Start();


            var unlinkedConstraints = cell.CellConstraints;
            AddConstraints(unlinkedConstraints);

            stopWatch.Stop();
            TimeSpan ts = stopWatch.Elapsed;
            string elapsedTime = String.Format("{0:00}:{1:00}:{2:00}.{3:000}", ts.Hours, ts.Minutes, ts.Seconds, ts.Milliseconds);
            Debug.WriteLine("(AddCellToGraph) RunTime " + elapsedTime);

        }

        private void RemoveCellFromGraph(Cell cell)
        {
            hasConstraintGraphChanged = true;
            Debug.WriteLine("(RemoveCellFromGraph) hasConstraintGraphChanged = " + hasConstraintGraphChanged);


            var stopWatch = new Stopwatch();
            stopWatch.Start();


            var cellConstraints = cell.CellConstraints;

            var containingComponents = new List<HashSet<HashSet<Edge>>>();

            foreach (var edgeConstraints in cellConstraints)
            {
                var edge = edgeConstraints.First();

                var containingComponent = FindContainingComponent(edge, ConstraintGraph);
                if (containingComponent == null)
                    continue;

                containingComponent.Remove(edgeConstraints);
                containingComponents.Add(containingComponent);
            }

            //check for changed components: empty? -> remove, not linked anymore? -> split
            foreach (var component in containingComponents)
            {
                if (component.Count == 1)
                    continue;

                if (component.Count == 0)
                {
                    ConstraintGraph.Remove(component);
                    continue;
                }

                var constraintsCopy = new HashSet<HashSet<Edge>>(component);
                foreach (var constraints in component)
                {
                    var doesOverlap = true;
                    foreach (var copy in constraintsCopy)
                    {
                        if (constraints.SetEquals(copy))
                            continue;

                        doesOverlap = constraints.Overlaps(copy);
                    }

                    if (doesOverlap)
                        continue;

                    ConstraintGraph.Add(new HashSet<HashSet<Edge>> { constraints });
                }

                ConstraintGraph.Remove(component);
            }

            stopWatch.Stop();
            TimeSpan ts = stopWatch.Elapsed;
            string elapsedTime = String.Format("{0:00}:{1:00}:{2:00}.{3:000}", ts.Hours, ts.Minutes, ts.Seconds, ts.Milliseconds);
            Debug.WriteLine("(RemoveCellFromGraph) RunTime " + elapsedTime);

        }

        private void AddConstraints(List<HashSet<Edge>> unlinkedConstraints)
        {
            var toVisit = new List<HashSet<Edge>>(unlinkedConstraints);

            while (toVisit.Count > 0)
            {
                var cellConstraints = toVisit[0];
                toVisit.RemoveAt(0);

                var component = new HashSet<HashSet<Edge>> { cellConstraints };
                var containingComponents = FindContainingComponents(cellConstraints, ConstraintGraph);

                if (containingComponents.Count > 0)
                {
                    foreach (var containingComponent in containingComponents)
                    {
                        component.UnionWith(containingComponent);
                        ConstraintGraph.Remove(containingComponent);
                    }
                }

                ConstraintGraph.Add(component);
            }

            Debug.WriteLine("current DoF: {0}", ConstraintGraph.Count);
        }


        //private void AddConstraints(List<HashSet<Edge>> unlinkedConstraints)
        //{
        //    var toVisit = new List<HashSet<Edge>>(unlinkedConstraints);
        //    //foreach (var component in ConstraintGraph)
        //    //    toVisit.AddRange(component);

        //    var edgesToVisit = new HashSet<Edge>();
        //    var visitedEdges = new HashSet<Edge>();

        //    while (toVisit.Count > 0)
        //    {
        //        var linkedEdgesOfCell = toVisit[0];
        //        toVisit.RemoveAt(0);

        //        var component = new HashSet<HashSet<Edge>> { linkedEdgesOfCell };
        //        edgesToVisit.UnionWith(linkedEdgesOfCell);
        //        var componentsToRemove = new List<HashSet<HashSet<Edge>>>();

        //        //var isContainedInExistingComponent = false;

        //        while (edgesToVisit.Count > 0)
        //        {
        //            var edge = edgesToVisit.First();
        //            edgesToVisit.Remove(edge);

        //            if (visitedEdges.Contains(edge))
        //                continue;

        //            visitedEdges.Add(edge);

        //            var containingComponent = FindContainingComponent(edge, ConstraintGraph);
        //            if (containingComponent == null)
        //                continue;

        //            //isContainedInExistingComponent = true;
        //            component.UnionWith(containingComponent);
        //            componentsToRemove.Add(containingComponent);

        //            //component.UnionWith(containingComponent);

        //            //foreach (var cellComponent in containingComponent)
        //            //{
        //            //    toVisit.Remove(cellComponent);
        //            //    //edgesToVisit.UnionWith(cellComponent);
        //            //}



        //            //var containingComponent = FindContainingComponent(edge, toVisit);
        //            //if (containingComponent == null)
        //            //    continue;

        //            ////var containingComponent = toVisit[componentIndex];
        //            //component.Add(containingComponent);
        //            //toVisit.Remove(containingComponent);
        //            ////toVisit.RemoveAt(componentIndex);

        //            //edgesToVisit.UnionWith(containingComponent);
        //        }

        //        //if(!isContainedInExistingComponent)
        //        ConstraintGraph.Add(component);

        //        foreach (var componentRemove in componentsToRemove)
        //            ConstraintGraph.Remove(componentRemove);
        //    }
        //}

        private Dictionary<HashSet<HashSet<Edge>>, List<Cell>> ComponentsCellLookup;


        public void UpdateConstraintsGraph()
        {
#if DEBUG
            var stopWatch = new Stopwatch();
            stopWatch.Start();
#endif

            var unlinkedConstraints = new List<HashSet<Edge>>();

            foreach (var cell in Cells)
            {
                foreach (var constraint in cell.CellConstraints)
                    unlinkedConstraints.Add(constraint);
            }

            ConstraintGraph = new List<HashSet<HashSet<Edge>>>();
            AddConstraints(unlinkedConstraints);
            UpdateComponentsCellLookup();
            hasConstraintGraphChanged = true;
            Debug.WriteLine("(UpdateConstraintsGraph) hasConstraintGraphChanged = " + hasConstraintGraphChanged);

#if DEBUG
            stopWatch.Stop();
            TimeSpan ts = stopWatch.Elapsed;
            string elapsedTime = String.Format("{0:00}:{1:00}:{2:00}.{3:0000}", ts.Hours, ts.Minutes, ts.Seconds, ts.Milliseconds);
            Debug.WriteLine("(UpdateConstraintsGraph) RunTime " + elapsedTime);
#endif
        }

        private void UpdateComponentsCellLookup()
        {
            ComponentsCellLookup = new Dictionary<HashSet<HashSet<Edge>>, List<Cell>>();

            foreach (var component in ConstraintGraph)
            {
                var containedCells = new List<Cell>();

                foreach (var cellConstraints in component)
                    foreach (var edge in cellConstraints)
                        containedCells.AddRange(edge.ContainingCells);
                //foreach (var cell in edge.ContainingCells)
                //    containedCells.Add(cell);

                ComponentsCellLookup.Add(component, containedCells.Distinct().ToList());
            }
        }

        public List<Cell> GetCandidatesForMerging(HashSet<HashSet<Edge>> component1, HashSet<HashSet<Edge>> component2)
        {
            var cells1 = ComponentsCellLookup[component1];
            var cells2 = ComponentsCellLookup[component2];

            var candidates = cells1.Intersect(cells2).ToList();
            return candidates;
        }

        //public bool MergeComponentsAt(HashSet<HashSet<Edge>> component1, HashSet<HashSet<Edge>> component2, Cell mergeAt)
        //{
        //    var success = false;


        //    return success;
        //}

        public List<Cell> GetCandidatesForSplitting(HashSet<HashSet<Edge>> component)
        {
            var allRigidCells = new List<Cell>();

            foreach (var cellConstraints in component)
            {
                if (cellConstraints.Count != 4)
                    continue;

                foreach (var edge in cellConstraints)
                {
                    foreach (var cell in edge.ContainingCells)
                    {
                        if (!(cell is RigidCell) || allRigidCells.Contains(cell))
                            continue;

                        allRigidCells.Add(cell);
                    }
                }
            }

            return allRigidCells;
        }

        public void TryMergeComponents(HashSet<HashSet<Edge>> component1, HashSet<HashSet<Edge>> component2)
        {
            if (component1.SetEquals(component2))
                return;

            component1.UnionWith(component2);
            ConstraintGraph.Remove(component2);

            var cells1 = ComponentsCellLookup[component1];
            var cells2 = ComponentsCellLookup[component2];
            cells1.AddRange(cells2.Distinct().ToList());

            ComponentsCellLookup.Remove(component2);
        }



        private HashSet<Edge> FindContainingComponent(Edge edge, List<HashSet<Edge>> components)
        {
            foreach (var component in components)
            {
                if (component.Contains(edge))
                    return component;
            }

            return null;
        }

        private HashSet<HashSet<Edge>> FindContainingComponent(Edge edge, List<HashSet<HashSet<Edge>>> components)
        {
            foreach (var component in components)
            {
                foreach (var cellComponent in component)
                {
                    if (cellComponent.Contains(edge))
                        return component;
                }
            }

            return null;
        }

        private HashSet<HashSet<Edge>> FindContainingComponent(HashSet<Edge> cellConstraints, List<HashSet<HashSet<Edge>>> constraints)
        {
            foreach (var component in constraints)
                if (component.Contains(cellConstraints))
                    return component;

            return null;
        }

        private List<HashSet<HashSet<Edge>>> FindContainingComponents(HashSet<Edge> cellConstraints, List<HashSet<HashSet<Edge>>> constraints)
        {
            var containingComponents = new List<HashSet<HashSet<Edge>>>();

            foreach (var component in constraints)
            {
                foreach (var cellComponent in component)
                {
                    foreach (var edge in cellConstraints)
                    {
                        if (cellComponent.Contains(edge))
                        {
                            containingComponents.Add(component);
                            break;
                        }
                    }
                }
            }

            return containingComponents;
        }

        private bool hasConstraintGraphChanged = false;

        private Dictionary<HashSet<HashSet<Edge>>, List<HashSet<Cell>>> NonShearingAreas = new Dictionary<HashSet<HashSet<Edge>>, List<HashSet<Cell>>>();
        private Dictionary<HashSet<HashSet<Edge>>, List<Cell[]>> NonShearingAreaBounds = new Dictionary<HashSet<HashSet<Edge>>, List<Cell[]>>();

        public List<HashSet<Cell>> GetNonShearingAreasInComponent(HashSet<HashSet<Edge>> component)
        {
            Debug.WriteLine("(GetNonShearingAreasInComponent) hasConstraintGraphChanged = " + hasConstraintGraphChanged);

            if (hasConstraintGraphChanged || !NonShearingAreas.ContainsKey(component))
                UpdateNonShearingAreas(component);

            return NonShearingAreas[component];
        }

        public void UpdateNonShearingAreas(HashSet<HashSet<Edge>> component)
        {
            // TODO generalize for different cell, now it only works for rigid & shear cells

            hasConstraintGraphChanged = false;
            Debug.WriteLine("(UpdateNonShearingAreas) hasConstraintGraphChanged = " + hasConstraintGraphChanged);

#if DEBUG
            var stopWatch = new Stopwatch();
            stopWatch.Start();
#endif

            var componentCells = ComponentsCellLookup[component];
            var nonShearingCells = new List<Cell>();

            // find all non-shearing cells
            foreach (var cell in componentCells)
            {
                var containsAllConstraints = true;

                foreach (var cellConstraint in cell.CellConstraints)
                {
                    if (component.Contains(cellConstraint))
                        continue;

                    containsAllConstraints = false;
                    break;
                }

                if (containsAllConstraints)
                    nonShearingCells.Add(cell);
            }


            // find non-shearing areas
            var rigidAreas = new List<HashSet<Cell>>();
            var rigidAreaBounds = new List<Cell[]>(2);

            // keeping _all_ compontes in list (if keeping only components with non-shearing areas in list, return here)
            if (nonShearingCells.Count <= 0)
                rigidAreaBounds = new List<Cell[]> { new Cell[] { } };

            while (nonShearingCells.Count > 0)
            {
                var currentCell = nonShearingCells[0];
                nonShearingCells.RemoveAt(0);

                var area = new HashSet<Cell> { currentCell };
                var min = currentCell;
                var max = currentCell;

                if (nonShearingCells.Count <= 0)
                {
                    rigidAreas.Add(area);
                    rigidAreaBounds.Add(new Cell[] { min, max });
                    break;
                }

                // find areas using flood fill
                Cell neighbor;
                var horizontalAreaCells = new HashSet<Cell> { currentCell };

                //east
                var cell = currentCell;
                do
                {
                    neighbor = TryMarkAreaCell(horizontalAreaCells, nonShearingCells, cell, 1, 0);
                    cell = neighbor;

                    if (neighbor != null)
                        max = neighbor;
                } while (neighbor != null);

                //west
                cell = currentCell;
                do
                {
                    neighbor = TryMarkAreaCell(horizontalAreaCells, nonShearingCells, cell, -1, 0);
                    cell = neighbor;

                    if (neighbor != null)
                        min = neighbor;
                } while (neighbor != null);

                foreach (var horizontalCell in horizontalAreaCells)
                {
                    //north
                    cell = horizontalCell;
                    do
                    {
                        neighbor = TryMarkAreaCell(area, nonShearingCells, cell, 0, 1);
                        cell = neighbor;

                        if (neighbor != null && neighbor.IndexVertex.X == max.IndexVertex.X)
                            max = neighbor;

                    } while (neighbor != null);

                    //south
                    cell = horizontalCell;
                    do
                    {
                        neighbor = TryMarkAreaCell(area, nonShearingCells, cell, 0, -1);
                        cell = neighbor;

                        if (neighbor != null && neighbor.IndexVertex.X == min.IndexVertex.X)
                            min = neighbor;

                        //here check for min
                    } while (neighbor != null);
                }

                area.UnionWith(horizontalAreaCells);
                rigidAreas.Add(area);
                rigidAreaBounds.Add(new Cell[] { min, max });
            }

            if (NonShearingAreas.ContainsKey(component))
            {
                NonShearingAreas[component] = rigidAreas;
                NonShearingAreaBounds[component] = rigidAreaBounds;
            }
            else
            {
                NonShearingAreas.Add(component, rigidAreas);
                NonShearingAreaBounds.Add(component, rigidAreaBounds);
            }

            var componentsToRemove = new List<HashSet<HashSet<Edge>>>();
            foreach (var comp in NonShearingAreas.Keys)
            {
                if (!ConstraintGraph.Contains(comp))
                    componentsToRemove.Add(comp);
            }

            foreach (var comp in componentsToRemove)
            {
                NonShearingAreas.Remove(comp);
                NonShearingAreaBounds.Remove(comp);
            }

#if DEBUG
            stopWatch.Stop();
            TimeSpan ts = stopWatch.Elapsed;
            string elapsedTime = String.Format("{0:00}:{1:00}:{2:00}.{3:0000}", ts.Hours, ts.Minutes, ts.Seconds, ts.Milliseconds);
            Debug.WriteLine("(UpdateNonShearingAreas) RunTime " + elapsedTime);
#endif
        }

        private Cell TryMarkAreaCell(HashSet<Cell> area, List<Cell> toVisit, Cell currentCell, int offsetX, int offsetY)
        {
            var neighborPosition = Vector.Add(currentCell.IndexVertex.ToInitialVector(), new Vector(offsetX, offsetY));
            var neighborCell = toVisit.Find(c => c.IndexVertex.Equals(neighborPosition));
            //var neighborCell = toVisit.Find(c => c.IndexVertex.X == currentCell.IndexVertex.X + offsetX && c.IndexVertex.Y == currentCell.IndexVertex.Y + offsetY);
            if (neighborCell == null)
                return null;

            area.Add(neighborCell);
            toVisit.Remove(neighborCell);

            return neighborCell;
        }

        private int[] GetFormatOfNonShearingArea(List<List<Cell>> rigidArea, HashSet<HashSet<Edge>> component)
        {
            //return [x, y]
            // (keep in mind that for 3D we need to do that for all 3 planes)
            throw new NotImplementedException();
        }

        public List<Cell> SplitComponentAt(HashSet<HashSet<Edge>> component, Cell cellToSplitAt, SplitMode splitMode, bool coupleComponents = false)
        {
            //can also split at Vector (without check that it is contained in component)

            List<Cell> changedCells = new List<Cell>();

            //var success = false;

            var nonShearingAreas = GetNonShearingAreasInComponent(component);
            var nonShearingAreaBounds = NonShearingAreaBounds[component];

            HashSet<Cell> area = null;
            var index = 0;

            for (index = 0; index < nonShearingAreas.Count; index++)
            {
                var areaCandidate = nonShearingAreas[index];
                if (!areaCandidate.Contains(cellToSplitAt))
                    continue;

                area = areaCandidate;
                break;
            }

            //cell does not exist in any non-shearing area of the given component
            if (area == null)
                return changedCells;

            //var areaBounds = nonShearingAreaBounds[index];
            //var lowerLeftBound = areaBounds[0].IndexVertex.ToInitialVector();
            //var upperRightBound = Vector.Add(areaBounds[1].IndexVertex.ToInitialVector(), new Vector(1,1));


            //corner splitmode not implemented yet
            if (splitMode == SplitMode.Corner)
                throw new NotImplementedException("SplitMode.Corner is not implemented yet.");


            //only horizontal & vertical splitmode
            if (cellToSplitAt is RigidCell)
            {
                AddCell(new ShearCell(), cellToSplitAt.IndexVertex.ToInitialVector(), new Size(1, 1), false);
                changedCells.Add(cellToSplitAt);
            }

            var offsetX = 0;
            var offsetY = 0;

            if (splitMode == SplitMode.Horizontal)
            {
                offsetX = 1;
                offsetY = 0;
            }
            else if (splitMode == SplitMode.Vertical)
            {
                offsetX = 0;
                offsetY = 1;
            }

            //var startX = cellToSplitAt.IndexVertex.X;
            //var startY = cellToSplitAt.IndexVertex.Y;
            Cell neighbor;

            //go in positive directions
            Cell cell = cellToSplitAt;
            do
            {
                var neighborPosition = Vector.Add(cell.IndexVertex.ToInitialVector(), new Vector(offsetX, offsetY));
                var isFound = TryFindByIndex(area, neighborPosition, out neighbor);
                if (!isFound)
                    continue;

                if (neighbor is RigidCell)
                {
                    AddCell(new ShearCell(), neighbor.IndexVertex.ToInitialVector(), new Size(1, 1), false);
                    changedCells.Add(neighbor);
                }

                cell = neighbor;
            } while (neighbor != null);

            //and in negative directions
            cell = cellToSplitAt;
            do
            {
                var neighborPosition = Vector.Add(cell.IndexVertex.ToInitialVector(), new Vector(offsetX * (-1), offsetY * (-1)));
                var isFound = TryFindByIndex(area, neighborPosition, out neighbor);
                if (!isFound)
                    continue;

                if (neighbor is RigidCell)
                {
                    AddCell(new ShearCell(), neighbor.IndexVertex.ToInitialVector(), new Size(1, 1), false);
                    changedCells.Add(neighbor);
                }

                cell = neighbor;
            } while (neighbor != null);

            UpdateConstraintsGraph();

            return changedCells;
        }

        public int GetSizeOfComponent(HashSet<HashSet<Edge>> component)
        {
            // can be # of cells:
            return component.Count;

            // or # of edges, # of rigid areas, etc.
        }

        public double GetRigidityOfComponent(HashSet<HashSet<Edge>> component)
        {
            // ratio rigid areas/cells

            var numberCellsOverall = (double)component.Count(); // get number edges instead?
            var numberNonShearingCells = 0;

            var nonShearingAreas = GetNonShearingAreasInComponent(component);
            foreach (var area in nonShearingAreas)
                numberNonShearingCells += area.Count;

            return numberNonShearingCells / numberCellsOverall;
        }


        //private Dictionary<HashSet<HashSet<Edge>>, List<HashSet<Cell>>> NonShearingAreas = new Dictionary<HashSet<HashSet<Edge>>, List<HashSet<Cell>>>();
        //private Dictionary<HashSet<HashSet<Edge>>, List<Cell[]>> NonShearingAreaBounds = new Dictionary<HashSet<HashSet<Edge>>, List<Cell[]>>();

        public bool GetClosestNonShearingAreaToAnchor(HashSet<HashSet<Edge>> component, Vertex anchoredVertex, out HashSet<Cell> closestNonShearingArea, out Cell closestCell, out Vertex closestVertex)
        {
            var nonShearingAreas = GetNonShearingAreasInComponent(component);
            var nonShearingAreaBounds = NonShearingAreaBounds[component];
            var anchor = anchoredVertex.ToInitialVector();

            var minDistance = double.PositiveInfinity;

            closestVertex = new Vertex(new Vector());
            closestCell = new RigidCell();
            closestNonShearingArea = new HashSet<Cell>();
            var success = false;

            for (var i = 0; i < nonShearingAreaBounds.Count; i++)
            {
                var areaBounds = nonShearingAreaBounds[i];
                if (areaBounds.Length <= 0)
                    continue;

                var area = nonShearingAreas[i].ToList();


                var lowerLeftCellIndex = areaBounds[0].IndexVertex.ToInitialVector();
                var closestAreaVector = lowerLeftCellIndex;
                var closestAreaCell = area.Find(c => c.IndexVertex == areaBounds[0].IndexVertex);
                var distance = Vector.Subtract(lowerLeftCellIndex, anchor).Length;

                var upperRightCellIndex = areaBounds[1].IndexVertex.ToInitialVector();
                var upperRight = Vector.Add(upperRightCellIndex, new Vector(1, 1));
                var tempVector = Vector.Subtract(upperRight, anchor);
                if (tempVector.Length < distance)
                {
                    closestAreaVector = upperRight;
                    closestAreaCell = area.Find(c => c.IndexVertex.Equals(upperRightCellIndex));
                    distance = tempVector.Length;
                }

                var upperLeftCellIndex = new Vector(lowerLeftCellIndex.X, upperRightCellIndex.Y);
                var upperLeft = Vector.Add(lowerLeftCellIndex, new Vector(0, 1));
                tempVector = Vector.Subtract(upperLeft, anchor);
                if (tempVector.Length < distance)
                {
                    closestAreaVector = upperLeft;
                    closestAreaCell = area.Find(c => c.IndexVertex.Equals(upperLeftCellIndex));
                    distance = tempVector.Length;
                }

                var lowerRightCellVector = new Vector(upperRightCellIndex.X, lowerLeftCellIndex.Y);
                var lowerRight = Vector.Add(lowerRightCellVector, new Vector(1, 0));
                tempVector = Vector.Subtract(lowerRight, anchor);
                if (tempVector.Length < distance)
                {
                    closestAreaVector = lowerRight;
                    closestAreaCell = area.Find(c => c.IndexVertex.Equals(lowerRightCellVector));
                    distance = tempVector.Length;
                }

                //if closer than current clostetVertex, set this as closest
                if (distance < minDistance)
                {
                    closestNonShearingArea = nonShearingAreas[i];
                    closestCell = closestAreaCell;
                    closestVertex = closestCell.CellVertices.Find(v => v.Equals(closestAreaVector));

                    success = true;
                }
            }

            return success;
        }

        //public void UpdateConstraintsGraph_old()
        //{
        //    var unlinkedConstraints = new List<HashSet<Edge>>();

        //    //foreach (var cell in Cells)
        //    //{
        //    //    foreach (var constraint in cell.CellConstraints)
        //    //        unlinkedConstraints.Add(constraint);
        //    //}

        //    foreach (var cell in Cells)
        //    {
        //        if (cell is RigidCell)
        //        {
        //            //ConstraintsGraph.Add(cell.CellEdges);
        //            unlinkedConstraints.Add(new HashSet<Edge>(cell.CellEdges));
        //        }
        //        else
        //        {
        //            //horizontal
        //            unlinkedConstraints.Add(new HashSet<Edge> { cell.CellEdges[1], cell.CellEdges[3] });
        //            //vertical
        //            unlinkedConstraints.Add(new HashSet<Edge> { cell.CellEdges[0], cell.CellEdges[2] });
        //        }
        //    }

        //    ConstraintGraph = new List<HashSet<HashSet<Edge>>>();

        //    var toVisit = new List<HashSet<Edge>>(unlinkedConstraints);

        //    var edgesToVisit = new HashSet<Edge>();
        //    var visitedEdges = new HashSet<Edge>();

        //    while (toVisit.Count > 0)
        //    {
        //        var linkedEdgesOfCell = toVisit[0];
        //        toVisit.RemoveAt(0);

        //        var component = new HashSet<HashSet<Edge>> { linkedEdgesOfCell };
        //        edgesToVisit.UnionWith(linkedEdgesOfCell);

        //        while (edgesToVisit.Count > 0)
        //        {
        //            var edge = edgesToVisit.First();
        //            edgesToVisit.Remove(edge);

        //            if (visitedEdges.Contains(edge))
        //                continue;

        //            visitedEdges.Add(edge);

        //            var containingComponent = FindContainingComponent(edge, toVisit);
        //            if (containingComponent == null)
        //                continue;

        //            //var containingComponent = toVisit[componentIndex];
        //            component.Add(containingComponent);
        //            toVisit.Remove(containingComponent);
        //            //toVisit.RemoveAt(componentIndex);

        //            edgesToVisit.UnionWith(containingComponent);
        //        }

        //        ConstraintGraph.Add(component);
        //    }

        //    UpdateComponentsCellLookup();
        //}


        public bool TryPickRandomComponents(out int componentIndex1, out int componentIndex2)
        {
            componentIndex1 = -1;
            componentIndex2 = -1;

            if (ConstraintGraph.Count < 2)
                return false;

            var random = new Random(DateTime.Now.Millisecond);
            componentIndex1 = random.Next(0, ConstraintGraph.Count);

            while (componentIndex2 < 0 || componentIndex2 == componentIndex1)
                componentIndex2 = random.Next(0, ConstraintGraph.Count);

            return true;
        }

        private bool TryFindByIndex(HashSet<Cell> set, Vector toFind, out Cell found)
        {
            foreach (var item in set)
            {
                if (item.IndexVertex.Equals(toFind))
                {
                    found = item;
                    return true;
                }
            }

            found = null;
            return false;
        }

        internal void SetNonShearingCells()
        {

            var newRigidCellPositions = new List<Vector>();
            for (var i = 0; i < Cells.Count; i++)
            {
                var cell = Cells[i];
                if (cell is RigidCell)
                    continue;

                foreach (var component in ConstraintGraph)
                {
                    var flattend = component.SelectMany(e => e);
                    var intersected = flattend.Intersect(cell.CellEdges).ToList();

                    if (intersected.Count == 4)
                    {
                        var cellPosition = cell.IndexVertex.ToInitialVector();
                        newRigidCellPositions.Add(cellPosition);
                    }
                }
            }

            foreach (var position in newRigidCellPositions)
            {
                AddCell(new RigidCell(), position, new Size(1, 1), false);
            }

            UpdateConstraintsGraph();
        }

        public string GetEncoding()
        {
            var encoding = new char[Cells.Count];

            for (var i = 0; i < Cells.Count; i++)
                encoding[i] = '1';

            var width = Vertices.Max(v => v.InitialX);
            var rigidCells = Cells.FindAll(cell => cell is RigidCell);

            foreach (var rigidCell in rigidCells)
            {
                var index = (int)(rigidCell.IndexVertex.X + rigidCell.IndexVertex.Y * width);
                encoding[index] = '0';
            }

            return new string(encoding);
        }

        public string GetEffectiveEncoding()
        {
            var encoding = new char[Cells.Count];

            for (var i = 0; i < Cells.Count; i++)
                encoding[i] = '1';

            var width = Vertices.Max(v => v.InitialX);
            foreach (var component in ConstraintGraph)
            {
                var nonShearingAreas = GetNonShearingAreasInComponent(component);

                foreach (var area in nonShearingAreas)
                {
                    foreach (var nonShearingCell in area)
                    {
                        var index = (int)(nonShearingCell.IndexVertex.X + nonShearingCell.IndexVertex.Y * width);
                        encoding[index] = '0';
                    }
                }
            }

            return new string(encoding);
        }
    }
}
