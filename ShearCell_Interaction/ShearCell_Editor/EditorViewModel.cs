using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using System.Xml;
using System.Xml.Serialization;
using HelixToolkit.Wpf;
using ShearCell_Data.View;

namespace ShearCell_Editor
{
    public class EditorViewModel : INotifyPropertyChanged
    {
        public static bool Is3DEnabled = false;

        private InteractionMode _interactionMode;
        public InteractionMode InteractionMode
        {
            get { return _interactionMode; }
            set
            {
                _interactionMode = value;
                RaisePropertyChanged("InteractionMode");
            }
        }

        private ElementSelection _selectedElement;
        public ElementSelection SelectedElement
        {
            get { return _selectedElement; }
            set
            {
                _selectedElement = value;
                RaisePropertyChanged("SelectedElement");
            }
        }

        public Dictionary<Point3D, Voxel> CellVisuals { get; set; }

        public List<Voxel> Voxels { get; set; }
        public Color CurrentColor { get; set; }
        public Model3DGroup Model { get; set; }

        public Dictionary<Model3D, Voxel> ModelToVoxel { get; private set; }
        public Dictionary<Model3D, Material> OriginalMaterial { get; private set; }

        public List<Model3D> Highlighted { get; set; }
        public Model3D PreviewModel { get; set; }
        public Model3D PreviewAnchorModel { get; set; }

        public Viewport3D Viewport { get; }

        public EditorViewModel(Viewport3D viewport)
        {
            Model = new Model3DGroup();
            Voxels = new List<Voxel>();
            Highlighted = new List<Model3D>();
            ModelToVoxel = new Dictionary<Model3D, Voxel>();

            OriginalMaterial = new Dictionary<Model3D, Material>();
            CurrentColor = Brushes.LightGray.Color;

            SelectedElement = ElementSelection.Freeform;

            Viewport = viewport;
        }

        private readonly XmlSerializer serializer = new XmlSerializer(typeof(List<Voxel>), new[] { typeof(Voxel) });

        public void Save(string fileName)
        {
            using (var w = XmlWriter.Create(fileName, new XmlWriterSettings { Indent = true }))
                serializer.Serialize(w, Voxels);
        }

        public bool TryLoad(string fileName)
        {
            try
            {
                using (var r = XmlReader.Create(fileName))
                {
                    var v = serializer.Deserialize(r);
                    Voxels = v as List<Voxel>;
                }
                UpdateModel();
                return true;
            }
            catch
            {
                return false;
            }
        }

        public void UpdateCellPreview(Point screenPosition)
        {
            bool isDeleteMode = Keyboard.IsKeyDown(Key.LeftShift);
            Vector3D normal;

            var source = FindSource(screenPosition, out normal);
            if (isDeleteMode)
            {
                PreviewVoxel(screenPosition, null);
                HighlightVoxel(source);
            }
            else
            {
                PreviewVoxel(screenPosition, source, normal);
                HighlightVoxel(null);
            }
        }

        public void RemoveCellPreview()
        {
            if (PreviewModel != null)
            {
                Model.Children.Remove(PreviewModel);
                PreviewModel = null;
            }
        }

        public void UpdateAnchorPreview(Point screenPosition)
        {
            ShowAnchorOnGrid(screenPosition);
            //HighlightVoxel(null);
        }

        public void RemoveAnchorPreview()
        {
            if (PreviewAnchorModel != null)
            {
                Model.Children.Remove(PreviewAnchorModel);
                PreviewAnchorModel = null;
            }
        }
        
        private Model3D FindSource(Point screenPosition, out Vector3D normal)
        {
            var hits = Viewport.FindHits(screenPosition);

            foreach (var h in hits)
            {
                if(!ModelToVoxel.ContainsKey(h.Model))
                    continue;

                //if (h.Model == PreviewModel)
                //    continue;

                normal = h.Normal;
                return h.Model;
            }

            normal = new Vector3D();
            return null;
        }


        public void UpdateModel()
        {
            Model.Children.Clear();
            ModelToVoxel.Clear();
            OriginalMaterial.Clear();

            foreach (var v in Voxels)
            {
                var voxelModel = CreateVoxelModel3D(v);
                OriginalMaterial.Add(voxelModel, voxelModel.Material);
                Model.Children.Add(voxelModel);
                ModelToVoxel.Add(voxelModel, v);
            }

            RaisePropertyChanged("Model");
        }

        private static GeometryModel3D CreateVoxelModel3D(Voxel v)
        {
            const double size = 0.98;

            var model3D = new GeometryModel3D();
            var meshBuilder = new MeshBuilder();

            meshBuilder.AddBox(new Point3D(0, 0, 0), size, size, size);
            model3D.Geometry = meshBuilder.ToMesh();
            model3D.Material = MaterialHelper.CreateMaterial(v.Color);
            model3D.Transform = new TranslateTransform3D(v.Position.X, v.Position.Y, v.Position.Z);
            return model3D;
        }

        public event PropertyChangedEventHandler PropertyChanged;

        protected void RaisePropertyChanged(string property)
        {
            var handler = PropertyChanged;
            if (handler != null)
            {
                handler(this, new PropertyChangedEventArgs(property));
            }
        }

        /// <summary>
        /// Adds the a voxel adjacent to the specified model.
        /// </summary>
        /// <param name="source">The source.</param>
        /// <param name="normal">The normal.</param>
        public void Add(Model3D source, Vector3D normal)
        {
            if (!ModelToVoxel.ContainsKey(source))
                return;

            var v = ModelToVoxel[source];
            AddVoxel(v.Position + normal);
        }

        /// <summary>
        /// Adds a voxel at the specified position.
        /// </summary>
        /// <param name="p">The p.</param>
        public void AddVoxel(Point3D p)
        {
            Voxels.Add(new Voxel(p, CurrentColor));
            UpdateModel();
        }

        /// <summary>
        /// Highlights the specified voxel model.
        /// </summary>
        /// <param name="model">The model.</param>
        public void HighlightVoxel(Model3D model)
        {
            foreach (var child in Model.Children)
            {
                var voxelModel = child as GeometryModel3D;
                if (voxelModel == null)
                    continue;

                if (!ModelToVoxel.ContainsKey(voxelModel))
                    continue;

                var voxel = ModelToVoxel[voxelModel];
                var originalMaterial = OriginalMaterial[voxelModel];

                // highlight color
                var highlightColor = Colors.Red;// Color.FromArgb(0x80, voxel.Color.R, voxel.Color.G, voxel.Color.B);
                voxelModel.Material = voxelModel == model ? MaterialHelper.CreateMaterial(highlightColor) : originalMaterial;
            }
        }

        /// <summary>
        /// Shows a preview voxel adjacent to the specified model (source).
        /// If source is null, hide the preview.
        /// </summary>
        /// <param name="source">The source.</param>
        /// <param name="normal">The normal.</param>
        public void PreviewVoxel(Point screenPosition, Model3D source, Vector3D normal = default(Vector3D))
        {
            if (PreviewModel != null)
                Model.Children.Remove(PreviewModel);

            PreviewModel = null;

            if (source == null)
                ShowPreviewOnGrid(screenPosition);
            else
                AddCell(source, normal);

        }

        public void AddCellOnGrid(Point screenPosition)
        {
            var ray = Viewport.Point2DtoRay3D(screenPosition);
            if (ray == null)
                return;

            var planeIntersection = ray.PlaneIntersection(new Point3D(0, 0, 0.5), new Vector3D(0, 0, 1));
            if (!planeIntersection.HasValue)
                return;

            var pRound = new Point3D(Math.Round(planeIntersection.Value.X), Math.Round(planeIntersection.Value.Y), 0);
            AddVoxel(pRound);
        }

        private void AddCell(Model3D source, Vector3D normal)
        {
            if (!ModelToVoxel.ContainsKey(source))
                return;

            var voxel = ModelToVoxel[source];
            var previewColor = Color.FromArgb(0x80, CurrentColor.R, CurrentColor.G, CurrentColor.B);
            var previewVoxel = new Voxel(voxel.Position + normal, previewColor);
            PreviewModel = CreateVoxelModel3D(previewVoxel);
            Model.Children.Add(PreviewModel);
        }

        private void ShowPreviewOnGrid(Point screenPosition)
        {
            var ray = Viewport.Point2DtoRay3D(screenPosition);
            if (ray == null)
                return;

            var planeIntersection = ray.PlaneIntersection(new Point3D(0, 0, 0.5), new Vector3D(0, 0, 1));
            if (!planeIntersection.HasValue)
                return;

            var pRound = new Point3D(Math.Round(planeIntersection.Value.X), Math.Round(planeIntersection.Value.Y), 0);
            var previewColor = Color.FromArgb(0x80, CurrentColor.R, CurrentColor.G, CurrentColor.B);

            var previewVoxel = new Voxel(pRound, previewColor);
            PreviewModel = CreateVoxelModel3D(previewVoxel);
            Model.Children.Add(PreviewModel);
        }
       
        private void ShowAnchorOnGrid(Point screenPosition)
        {
            if (PreviewAnchorModel != null)
                Model.Children.Remove(PreviewAnchorModel);

            PreviewAnchorModel = null;

            var ray = Viewport.Point2DtoRay3D(screenPosition);
            if (ray == null)
                return;

            var planeIntersection = ray.PlaneIntersection(new Point3D(0, 0, 0.5), new Vector3D(0, 0, 1));
            if (!planeIntersection.HasValue)
                return;
           
            var pRound = new Point3D(Math.Round(planeIntersection.Value.X), Math.Round(planeIntersection.Value.Y), 0);
            var offset = pRound - planeIntersection.Value;

            pRound.X += offset.X > 0 ? -.5 : .5;
            pRound.Y += offset.Y > 0 ? -.5 : .5;
            
            var previewColor = Color.FromArgb(0x80, CurrentColor.R, CurrentColor.G, CurrentColor.B);

            const double radius = 0.2;

            var model3D = new GeometryModel3D();
            var meshBuilder = new MeshBuilder();

            meshBuilder.AddSphere(pRound, radius);
            model3D.Geometry = meshBuilder.ToMesh();
            model3D.Material = MaterialHelper.CreateMaterial(previewColor);

            PreviewAnchorModel = model3D;
            Model.Children.Add(PreviewAnchorModel);
        }

        //public void PreviewVoxel(Model3D source, Vector3D normal = default(Vector3D))
        //{
        //    if (PreviewModel != null)
        //        Model.Children.Remove(PreviewModel);

        //    PreviewModel = null;
        //    if (source == null)
        //        return;

        //    if (!ModelToVoxel.ContainsKey(source))
        //        return;

        //    var v = ModelToVoxel[source];
        //    var previewColor = Color.FromArgb(0x80, CurrentColor.R, CurrentColor.G, CurrentColor.B);
        //    var pv = new Voxel(v.Position + normal, previewColor);
        //    PreviewModel = CreateVoxelModel3D(pv);
        //    Model.Children.Add(PreviewModel);
        //}

        public void Remove(Model3D model)
        {
            if (!ModelToVoxel.ContainsKey(model))
                return;
            var v = ModelToVoxel[model];
            Voxels.Remove(v);
            UpdateModel();
        }

        public void Clear()
        {
            Voxels.Clear();
            UpdateModel();
        }
    }
}