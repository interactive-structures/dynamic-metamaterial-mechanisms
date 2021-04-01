using System;
using System.Collections.Generic;
using System.IO;
using System.Windows;
using ShearCell_Interaction.Helper;
using ShearCell_Interaction.Simulation;
using ShearCell_Interaction.View;

namespace ShearCell_Interaction.Model
{
    public class ModelSerializer
    {

        //TODO save headers in array for deserializing


        public static void WriteToFile(string path, MetamaterialModel model, ViewModel viewModel = null, bool writeOutputPath = true)
        {
            model.ResetDeformation();
            model.MoveToCoordinateOrigin();

            if (viewModel != null)
            {
                viewModel.Redraw();

                viewModel.HighlightInputVertex();
                viewModel.DrawInputPath();
                viewModel.HighlightOutputVertex();
                viewModel.DrawOutputPath();
            }

            using (var file = new StreamWriter(path))
            {
                var anchors = model.GetAnchors();

                file.WriteLine("#num_vertices #num_cells #num_anchors #index_inputvertex #num_inputpoints #index_outputvertex #num_outputpoints");

                file.Write(model.Vertices.Count + " ");
                file.Write(model.Cells.Count + " ");
                file.Write(anchors.Count + " ");

                file.Write(model.Vertices.IndexOf(model.InputVertex) + " ");
                file.Write(model.InputPath.Count + " ");

                if(writeOutputPath)
                {
                    file.Write(model.Vertices.IndexOf(model.OutputVertex) + " ");
                    file.WriteLine(model.OutputPath.Count + " ");

                }
                else
                {
                    file.Write("-1 ");
                    file.WriteLine("0 ");
                }


                file.WriteLine();

                file.WriteLine("#vertices");
                foreach (var vertex in model.Vertices)
                    file.WriteLine("{0} {1}", vertex.InitialX, vertex.InitialY);

                file.Flush();
                file.WriteLine();

                file.WriteLine("#anchors");
                foreach (var anchor in anchors)
                    file.WriteLine(model.Vertices.IndexOf(anchor));


                file.WriteLine();

                file.WriteLine("#cells [type s=shear r=rigid]");
                foreach (var cell in model.Cells)
                {
                    var cellType = cell is ShearCell ? "s" : "r";
                    file.Write(cellType + " ");

                    foreach (var cellVertex in cell.CellVertices)
                        file.Write(model.Vertices.IndexOf(cellVertex) + " ");

                    file.WriteLine();
                }

                file.Flush();
                file.WriteLine();

                file.WriteLine("#input path");
                foreach (var pathPoint in model.InputPath)
                    file.WriteLine("{0} {1}", pathPoint.X, pathPoint.Y);


                file.Flush();
                file.WriteLine();

                file.WriteLine("#output path");
                if(writeOutputPath)
                {
                    foreach (var pathPoint in model.OutputPath)
                        file.WriteLine("{0} {1}", pathPoint.X, pathPoint.Y);
                }

                if (viewModel != null)
                {
                    file.Flush();
                    file.WriteLine();

                    file.WriteLine("#tracing points");

                    foreach (var tracingPoint in viewModel.TracedVertices.Keys)
                    {
                        var point = tracingPoint.ToVector();
                        file.WriteLine("{0} {1}", point.X, point.Y);
                    }
                }
            }

        }

        public static void ReadFromFile(string filename, MetamaterialModel model, ViewModel viewModel = null)
        {
            using (var file = new StreamReader(filename))
            {
                string line;
                var inputVertexIndex = -1;
                var outputVertexIndex = -1;

                while ((line = file.ReadLine()) != null)
                {
                    if (string.IsNullOrWhiteSpace(line))
                        continue;

                    if (line.Contains("#num_vertices #num_cells #num_anchors #index_inputvertex #num_inputpoints #index_outputvertex #num_outputpoints"))
                    {
                        line = file.ReadLine();
                        var sums = line.Split(' ');
                        inputVertexIndex = int.Parse(sums[3]);
                        outputVertexIndex = int.Parse(sums[5]);
                    }

                    else if (line.Contains("#vertices"))
                    {
                        while ((line = file.ReadLine()) != null)
                        {
                            if (string.IsNullOrWhiteSpace(line) || line.Contains("#"))
                                break;

                            var coordinates = line.Split(' ');
                            var vertex = new Vertex(new Vector(double.Parse(coordinates[0]), double.Parse(coordinates[1])));
                            model.Vertices.Add(vertex);
                        }
                    }

                    else if (line.Contains("#anchors"))
                    {
                        while ((line = file.ReadLine()) != null)
                        {
                            if (string.IsNullOrWhiteSpace(line) || line.Contains("#"))
                                break;

                            int anchorIndex;
                            var success = int.TryParse(line, out anchorIndex);

                            if (success)
                                model.Vertices[anchorIndex].IsAnchor = true;

                        }
                    }

                    else if (line.Contains("#cells"))
                    {
                        while ((line = file.ReadLine()) != null)
                        {
                            if (string.IsNullOrWhiteSpace(line) || line.Contains("#"))
                                break;

                            var cellSpecifications = line.Split(' ');

                            Cell cell;
                            if (cellSpecifications[0] == "s")
                                cell = new ShearCell();
                            else if (cellSpecifications[0] == "r")
                                cell = new RigidCell();
                            else
                                break;

                            var vertices = new List<Vertex>();

                            for (var i = 1; i < cellSpecifications.Length; i++)
                            {
                                int vertexIndex;
                                var success = int.TryParse(cellSpecifications[i], out vertexIndex);

                                if (success)
                                    vertices.Add(model.Vertices[vertexIndex]);
                            }

                            model.AddCellFromVertices(cell, vertices);
                        }
                    }

                    else if (line.Contains("#input path"))
                    {
                        while ((line = file.ReadLine()) != null)
                        {
                            if (string.IsNullOrWhiteSpace(line) || line.Contains("#"))
                                break;

                            var coordinates = line.Split(' ');
                            //InputPath.Add(new Vector(double.Parse(coordinates[0]), double.Parse(coordinates[1])));

                            double x;
                            var successX = double.TryParse(coordinates[0], out x);
                            double y;
                            var successY = double.TryParse(coordinates[1], out y);

                            if (successX && successY)
                                model.InputPath.Add(new Vector(x, y));
                        }
                    }

                    else if (line.Contains("#output path"))
                    {
                        while ((line = file.ReadLine()) != null)
                        {
                            if (string.IsNullOrWhiteSpace(line) || line.Contains("#"))
                                break;

                            var coordinates = line.Split(' ');
                            //InputPath.Add(new Vector(double.Parse(coordinates[0]), double.Parse(coordinates[1])));

                            double x;
                            var successX = double.TryParse(coordinates[0], out x);
                            double y;
                            var successY = double.TryParse(coordinates[1], out y);

                            if (successX && successY)
                                model.OutputPath.Add(new Vector(x, y));
                        }
                    }

                    else if (line.Contains("#tracing points") && viewModel != null)
                    {
                        while ((line = file.ReadLine()) != null)
                        {
                            if (string.IsNullOrWhiteSpace(line) || line.Contains("#"))
                                break;

                            var coordinates = line.Split(' ');
                            var vertex = new Vertex(new Vector(double.Parse(coordinates[0]), double.Parse(coordinates[1])));

                            var existingVertex = model.Vertices.Find(current => MathHelper.IsEqualDouble(current.X, vertex.X) && MathHelper.IsEqualDouble(current.Y, vertex.Y));

                            if (existingVertex == null)
                                continue;

                            viewModel.AddTracingPoint(existingVertex);
                        }
                    }
                }

                if (inputVertexIndex != -1)
                    model.InputVertex = model.Vertices[inputVertexIndex];

                if (outputVertexIndex != -1)
                    model.OutputVertex = model.Vertices[outputVertexIndex];
            }
        }

    }
}
