using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Accord;
using Accord.Math;
using GoogleARCore;
using GoogleARCore.Examples.AugmentedImage;
using System.IO;

public class ReadScanController : MonoBehaviour {

    public GameObject Scanner;
    //Path for reading point cloud from device
    private string path;

    //Current Session marker corner positions
    private UnityEngine.Vector3 FrameLowerLeft;
    private UnityEngine.Vector3 FrameLowerRight;
    private UnityEngine.Vector3 FrameUpperLeft;
    private UnityEngine.Vector3 FrameUpperRight;

    //Read Session marker corner positions
    private UnityEngine.Vector3 ReadFrameLowerLeft;
    private UnityEngine.Vector3 ReadFrameLowerRight;
    private UnityEngine.Vector3 ReadFrameUpperLeft;
    private UnityEngine.Vector3 ReadFrameUpperRight;

    //Augmented Image variables
    public AugmentedImageVisualizer AugmentedImageVisualizerPrefab;
    private Dictionary<int, AugmentedImageVisualizer> m_Visualizers
            = new Dictionary<int, AugmentedImageVisualizer>();

    private List<AugmentedImage> m_TempAugmentedImages = new List<AugmentedImage>();
    AugmentedImageVisualizer visualizer;

    //Scanner positions
    private List<UnityEngine.Vector3> ScannerPos = new List<UnityEngine.Vector3>();

    // Transformation Matrix between simulation and read simulation sessions
    UnityEngine.Matrix4x4 TransformationMatrix;

    void Start () {

        AndroidJavaClass jc = new AndroidJavaClass("android.os.Environment");
        path = jc.CallStatic<AndroidJavaObject>("getExternalStoragePublicDirectory", jc.GetStatic<string>("DIRECTORY_DCIM")).Call<string>("getAbsolutePath");
        path = path.Substring(0, path.IndexOf("/DCIM")) + "/Android/data/" + Application.identifier;
        Debug.Log("Path is" + path);
    }

    // Update is called once per frame
    void Update()
    {
        // Exit the app when the 'back' button is pressed.
        if (Input.GetKey(KeyCode.Escape))
        {
            Application.Quit();
        }
       
        // Get updated augmented images for this frame.
        Session.GetTrackables<AugmentedImage>(m_TempAugmentedImages, TrackableQueryFilter.Updated);

        // Create visualizers and anchors for updated augmented images that are tracking and do not previously
        // have a visualizer. Remove visualizers for stopped images.
        foreach (var image in m_TempAugmentedImages)
        {
            visualizer = null;
            m_Visualizers.TryGetValue(image.DatabaseIndex, out visualizer);
            if (image.TrackingState == TrackingState.Tracking && visualizer == null)
            {
                // Create an anchor to ensure that ARCore keeps tracking this augmented image.

                Anchor anchor = image.CreateAnchor(image.CenterPose);
                visualizer = (AugmentedImageVisualizer)Instantiate(AugmentedImageVisualizerPrefab, anchor.transform);
                visualizer.Image = image;
                m_Visualizers.Add(image.DatabaseIndex, visualizer);

                Debug.LogWarning("First if is called and visualizer is created");

            }

            else if (image.TrackingState == TrackingState.Stopped && visualizer != null)
            {

                m_Visualizers.Remove(image.DatabaseIndex);
                GameObject.Destroy(visualizer.gameObject);
            }         
        }
        if (m_Visualizers.Count > 0)
        {
            FrameLowerLeft = visualizer.FrameLowerLeft.transform.position;
            FrameLowerRight = visualizer.FrameLowerRight.transform.position;
            FrameUpperLeft = visualizer.FrameUpperLeft.transform.position;
            FrameUpperRight = visualizer.FrameUpperRight.transform.position;
        }
    }

    public void LoadMap()
    {
        Debug.Log("Load Map is called");
        if(File.Exists(path+"/Map.txt"))
        {
            Debug.Log("File found");
        }
        try
        {
            var fs = new FileStream(path + "/Map.txt", FileMode.Open, FileAccess.Read);
            using (var bs = new BufferedStream(fs))
            using (var sr = new StreamReader(bs))
            {
                int numOfScanner = int.Parse(sr.ReadLine());
                Debug.Log("Number of scanners are" + numOfScanner);
              
               
                ReadFrameLowerLeft.x = float.Parse(sr.ReadLine());
                ReadFrameLowerLeft.y = float.Parse(sr.ReadLine());
                ReadFrameLowerLeft.z = float.Parse(sr.ReadLine());

                ReadFrameLowerRight.x = float.Parse(sr.ReadLine());
                ReadFrameLowerRight.y = float.Parse(sr.ReadLine());
                ReadFrameLowerRight.z = float.Parse(sr.ReadLine());

                
                ReadFrameUpperLeft.x = float.Parse(sr.ReadLine());
                ReadFrameUpperLeft.y = float.Parse(sr.ReadLine());
                ReadFrameUpperLeft.z = float.Parse(sr.ReadLine());

               
                ReadFrameUpperRight.x = float.Parse(sr.ReadLine());
                ReadFrameUpperRight.y = float.Parse(sr.ReadLine());
                ReadFrameUpperRight.z = float.Parse(sr.ReadLine());

                Debug.Log("Read lower left is" + ReadFrameLowerLeft);
                Debug.Log("Read lower right is" + ReadFrameLowerRight);
                Debug.Log("Read upper left is" + ReadFrameUpperLeft);
                Debug.Log("Read upper right is" + ReadFrameUpperRight);
               
                for(int i = 0; i < numOfScanner; i++)
                {
                   
                    var x = float.Parse(sr.ReadLine());
                    var y = float.Parse(sr.ReadLine());
                    var z = float.Parse(sr.ReadLine());
                    ScannerPos.Add(new UnityEngine.Vector3(x, y, z));
                }
            }
        }
        catch(FileNotFoundException ex)
        {
            Debug.LogError("File not found");
        }
        ApplyTransformation(out TransformationMatrix);
        Pose pose;
        int count = 1;
        foreach(var scanner in ScannerPos)
        {
            pose.position = TransformationMatrix.MultiplyPoint(scanner);
            pose.rotation = Quaternion.identity;
            var obj = Instantiate(Scanner, pose.position,pose.rotation);
            obj.transform.Rotate(-90, 0, 90);
            obj.transform.GetChild(0).GetComponent<TextMesh>().text = count.ToString();
            count++;
            Anchor anchor = Session.CreateAnchor(pose);
            obj.transform.parent = anchor.transform; 
        }

    }

    private Matrix3x3 CovarianceMatrixStep(Accord.Math.Vector3 difSetA, Accord.Math.Vector3 difSetB)
    {
        Matrix3x3 M;
        M.V00 = difSetA.X * difSetB.X;
        M.V01 = difSetA.X * difSetB.Y;
        M.V02 = difSetA.X * difSetB.Z;

        M.V10 = difSetA.Y * difSetB.X;
        M.V11 = difSetA.Y * difSetB.Y;
        M.V12 = difSetA.Y * difSetB.Z;

        M.V20 = difSetA.Z * difSetB.X;
        M.V21 = difSetA.Z * difSetB.Y;
        M.V22 = difSetA.Z * difSetB.Z;


        return M;
    }

    // Converting Unity.Vector3 to Accord.Vector3
    private Accord.Math.Vector3 UnitytoAccord(UnityEngine.Vector3 pos)
    {
        Accord.Math.Vector3 posTransformed = new Accord.Math.Vector3();
        posTransformed.X = pos.x;
        posTransformed.Y = pos.y;
        posTransformed.Z = pos.z;

        return posTransformed;
    }

    // Converting  Accord.Vector3 to Unity.Vector3 
    private UnityEngine.Vector3 AccordtoUnity(Accord.Math.Vector3 pos)
    {
        UnityEngine.Vector3 posTransformed = new UnityEngine.Vector3();
        posTransformed.x = pos.X;
        posTransformed.y = pos.Y;
        posTransformed.z = pos.Z;

        return posTransformed;
    }

    private Matrix3x3 NegativeMatrix(Matrix3x3 m)
    {
        m.V00 *= (-1);
        m.V01 *= (-1);
        m.V02 *= (-1);
        m.V10 *= (-1);
        m.V11 *= (-1);
        m.V12 *= (-1);
        m.V20 *= (-1);
        m.V21 *= (-1);
        m.V22 *= (-1);

        return m;
    }

    // Creating Unity Transformation matrix using 3x3 Rotation matrix and translation vector acquired from RigidTransform

    private UnityEngine.Matrix4x4 AccordToUnityMatrix(UnityEngine.Matrix4x4 UnityM, Accord.Math.Matrix3x3 RotationM, Accord.Math.Vector3 Trans)
    {
        //right
        UnityM.m00 = -RotationM.V00;
        UnityM.m10 = -RotationM.V10;
        UnityM.m20 = -RotationM.V20;
        //up
        UnityM.m01 = RotationM.V02;
        UnityM.m11 = RotationM.V12;
        UnityM.m21 = RotationM.V22;
        //forward
        UnityM.m02 = RotationM.V01;
        UnityM.m12 = RotationM.V11;
        UnityM.m22 = RotationM.V21;


        UnityM.m03 = Trans.X;
        UnityM.m13 = Trans.Y;
        UnityM.m23 = Trans.Z;

        UnityM.m30 = 0;
        UnityM.m31 = 0;
        UnityM.m32 = 0;
        UnityM.m33 = 1;

        return UnityM;
    }


    private Accord.Math.Vector3 CalculateCentroid(Accord.Math.Vector3 first, Accord.Math.Vector3 second, Accord.Math.Vector3 third, Accord.Math.Vector3 fourth)
    {
        return (first + second + third + fourth) / 4;
    }


    private void ApplyTransformation(out UnityEngine.Matrix4x4 Transformation)
    {
        //Calculating Centroids from both coordinate system

        Accord.Math.Vector3 centroidA = CalculateCentroid(UnitytoAccord(ReadFrameLowerLeft), UnitytoAccord(ReadFrameLowerRight), UnitytoAccord(ReadFrameUpperLeft),UnitytoAccord(ReadFrameUpperRight));
        Accord.Math.Vector3 centroidB = CalculateCentroid(UnitytoAccord(FrameLowerLeft), UnitytoAccord(FrameLowerRight), UnitytoAccord(FrameUpperLeft), UnitytoAccord(FrameUpperRight));

        Matrix3x3 H = CovarianceMatrixStep(UnitytoAccord(ReadFrameLowerLeft) - centroidA, UnitytoAccord(FrameLowerLeft) - centroidB)
            + CovarianceMatrixStep(UnitytoAccord(ReadFrameLowerRight) - centroidA, UnitytoAccord(FrameLowerRight) - centroidB)
            + CovarianceMatrixStep(UnitytoAccord(ReadFrameUpperLeft) - centroidA, UnitytoAccord(FrameUpperLeft) - centroidB)
            + CovarianceMatrixStep(UnitytoAccord(ReadFrameUpperRight) - centroidA, UnitytoAccord(FrameUpperRight) - centroidB);
        Matrix3x3 U;
        Accord.Math.Vector3 E;
        Matrix3x3 V;
        Matrix3x3 R;
        H.SVD(out U, out E, out V);

        R = V * U.Transpose();
        Debug.Log("Row  " + R.GetRow(0));
        Debug.Log("Row  " + R.GetRow(1));
        Debug.Log("Row  " + R.GetRow(2));


        if (R.Determinant < 0)
        {
            V.V02 = (-V.V02);
            V.V12 = (-V.V12);
            V.V22 = (-V.V22);
            R = V * U.Transpose();
            Debug.LogWarning("Reflection case");

        }
        Accord.Math.Vector3 Translation;
        Translation = (NegativeMatrix(R) * centroidA + centroidB);
        Debug.Log("Translation is" + Translation);
        Transformation = UnityEngine.Matrix4x4.identity;
        Transformation = AccordToUnityMatrix(Transformation, R, Translation);

        Debug.Log("Trans" + Transformation.GetRow(0));
        Debug.Log("Trans" + Transformation.GetRow(1));
        Debug.Log("Trans" + Transformation.GetRow(2));
        Debug.Log("Trans" + Transformation.GetRow(3));

        Transformation.SetTRS(AccordtoUnity(Translation), Quaternion.LookRotation(Transformation.GetColumn(1),
             Transformation.GetColumn(2)), UnityEngine.Vector3.one);


    }
}
