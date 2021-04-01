using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;

public enum CollisionType
{
    SphereSphere , SphereCapsule , SphereOBB , CapsuleLockCapsuleLock , CaspuleCapsule , OBBOBB , SphereTriangle
}

public class CollisionTester : MonoBehaviour
{
    public CollisionType collisionType;
    public bool resolveCollisions;
    public bool drawMarkers;

    public float3 posA;
    public float3 posB;

    public float3 rotA;
    public float3 rotB;

    public float3 halfSizeA;
    public float3 halfSizeB;

    private GameObject shapeA;
    private GameObject shapeB;
    private GameObject closestPointMarker;
    private GameObject[] vertsMarkers;
    private Color normalColor = Color.black;
    private Color collisionColor = Color.red;
    private Color markerColor = Color.blue;

    private CollisionType lastCollisionType;
    private bool collisionFlag = false;

    private void Start()
    {
        SetupScene();
    }
    private void Update()
    {
        collisionFlag = false;

        if ( collisionType != lastCollisionType )
        {
            lastCollisionType = collisionType;
            SetupScene();
        }

        switch ( collisionType )
        {
            case CollisionType.SphereSphere:
            HandleSpheres();
            break;
            case CollisionType.SphereCapsule:
            HandleSphereCapsule();
            break;
            case CollisionType.SphereOBB:
            HandleSphereOBB();
            break;
            case CollisionType.CapsuleLockCapsuleLock:
            HandleCapsulesLock();
            break;
            case CollisionType.CaspuleCapsule:
            HandleCapsuleCapsule();
            break;
            case CollisionType.OBBOBB:
            HandleOBBOBB();
            break;
            case CollisionType.SphereTriangle:
            HandleSphereTriangle();
            break;
            default:
            Debug.Log( "No CollisionType Selected" );
            break;
        }

        if ( collisionFlag )
        {
            shapeA.GetComponent<MeshRenderer>().material.color = collisionColor;
            shapeB.GetComponent<MeshRenderer>().material.color = collisionColor;
        }
        else
        {
            shapeA.GetComponent<MeshRenderer>().material.color = normalColor;
            shapeB.GetComponent<MeshRenderer>().material.color = normalColor;
        }
    }

    private void SetupScene()
    {
        ResetTransform();

        if ( shapeA )
            Destroy( shapeA );
        if ( shapeB )
            Destroy( shapeB );
        if ( closestPointMarker )
            Destroy( closestPointMarker );

        switch ( collisionType )
        {
            case CollisionType.SphereSphere:
            SetupSpheres();
            break;
            case CollisionType.SphereCapsule:
            SetupSphereCapsule();
            break;
            case CollisionType.SphereOBB:
            SetupSphereBox();
            break;
            case CollisionType.CapsuleLockCapsuleLock:
            SetupLockedCapsule();
            break;
            case CollisionType.CaspuleCapsule:
            SetupCapsuleCapsule();
            break;
            case CollisionType.OBBOBB:
            SetupBoxBox();
            break;
            case CollisionType.SphereTriangle:
            SetupPointTriangle();
            break;
            default:
            Debug.Log( "No CollisionType Selected" );
            break;
        }

        if ( shapeA )
        {
            Material material = new Material( Shader.Find( "Standard" ) );
            material.color = normalColor;
            shapeA.GetComponent<MeshRenderer>().material = material;
        }
        if ( shapeB )
        {
            Material material = new Material( Shader.Find( "Standard" ) );
            material.color = normalColor;
            shapeB.GetComponent<MeshRenderer>().material = material;
        }

        if ( drawMarkers )
        {
            closestPointMarker = GameObject.CreatePrimitive( PrimitiveType.Sphere );
            Material material = new Material( Shader.Find( "Standard" ) );
            material.color = markerColor;
            closestPointMarker.GetComponent<MeshRenderer>().material = material;
        }
    }
    private void SetupSpheres()
    {
        shapeA = GameObject.CreatePrimitive( PrimitiveType.Sphere );
        shapeB = GameObject.CreatePrimitive( PrimitiveType.Sphere );
    }
    private void SetupLockedCapsule()
    {
        shapeA = GameObject.CreatePrimitive( PrimitiveType.Capsule );
        shapeB = GameObject.CreatePrimitive( PrimitiveType.Capsule );
    }
    private void SetupCapsuleCapsule()
    {
        shapeA = GameObject.CreatePrimitive( PrimitiveType.Capsule );
        shapeB = GameObject.CreatePrimitive( PrimitiveType.Capsule );
    }
    private void SetupBoxBox()
    {
        shapeA = GameObject.CreatePrimitive( PrimitiveType.Cube );
        shapeB = GameObject.CreatePrimitive( PrimitiveType.Cube );
    }
    private void SetupSphereCapsule()
    {
        shapeA = GameObject.CreatePrimitive( PrimitiveType.Sphere );
        shapeB = GameObject.CreatePrimitive( PrimitiveType.Capsule );
    }
    private void SetupSphereBox()
    {
        shapeA = GameObject.CreatePrimitive( PrimitiveType.Sphere );
        shapeB = GameObject.CreatePrimitive( PrimitiveType.Cube );
    }
    private void SetupPointTriangle()
    {
        shapeA = GameObject.CreatePrimitive( PrimitiveType.Sphere );
        shapeB = new GameObject();
        shapeB.transform.position = posB;
        shapeB.transform.rotation = quaternion.EulerXYZ( rotB.x , rotB.y + ColPhysics.DegreesToRadians( 180 ) , rotB.z );
        shapeB.transform.rotation = quaternion.EulerXYZ( rotB.x , rotB.y + ColPhysics.DegreesToRadians( 180 ) , rotB.z );
        SetupTriangleMesh();
        vertsMarkers = new GameObject[ 3 ];
        for ( int i = 0; i < vertsMarkers.Length; i++ )
        {
            vertsMarkers[ i ] = GameObject.CreatePrimitive( PrimitiveType.Sphere );
            vertsMarkers[ i ].transform.localScale = new float3( 0.08f , 0.08f , 0.08f );
        }
    }
    private void ResetTransform()
    {
        posA = new float3( -5 , 0 , 0 );
        posB = float3.zero;

        rotA = float3.zero;
        rotB = float3.zero;

        halfSizeA = new float3( 1 , 1 , 1 );
        halfSizeB = new float3( 1 , 1 , 1 );
    }

    // SPHERE SPHERE
    private void HandleSpheres()
    {
        CollisionSphere();
        DrawSpheres();
    }
    private void CollisionSphere()
    {
        float3 pa = posA;
        float3 pb = posB;
        float ra = halfSizeA.x;
        float rb = halfSizeB.x;

        if ( ColPhysics.SpheresIntersect( pa , pb , ra , rb , out float distance ) )
        {
            collisionFlag = true;

            if ( resolveCollisions )
            {
                ColPhysics.ResolveSphereCollision( ref pa , ref pb , ra , rb , distance );

                posA = pa;
                posB = pb;
            }
        }
    }

    // SPHERE CAPSULE
    private void HandleSphereCapsule()
    {
        CollisionSphereCapsule();
        DrawSphereCapsule();
    }
    private void CollisionSphereCapsule()
    {
        float3 spherePos = posA;
        float sphereRadius = halfSizeA.x;

        float3 capsulePos = posB;
        float3 capsuleRot = rotB;
        float capsuleLength = halfSizeB.y * 2;
        float capsuleRadius = halfSizeB.x;

        if ( ColPhysics.SphereIntersectsCapsule( spherePos , sphereRadius , capsulePos , capsuleRot , capsuleLength , capsuleRadius , out float distance ) )
        {
            collisionFlag = true;

            if ( resolveCollisions )
            {
                ColPhysics.ResolveSphereCollision( ref spherePos , ref capsulePos , sphereRadius , capsuleRadius , distance );

                posA = spherePos;
                posB = capsulePos;
            }
        }
    }

    // SPHERE BOX
    private void HandleSphereOBB()
    {
        CollisionSphereOBB();
        DrawSphereBox();
    }
    private void CollisionSphereOBB()
    {
        float3 spherePos = posA;
        float3 obbPos = posB;
        float3 obbRot = rotB;
        float3 obbHalfSize = halfSizeB;
        float sphereRadius = halfSizeA.x;

        FixedList128<float3> vertices = ColPhysics.GetAABBVerticesOBB( obbPos , obbHalfSize ); // calculated once at startup forever stored with entity
        vertices = ColPhysics.GetRotatedVerticesOBB( vertices , obbPos , obbRot );
        FixedList128<float> extents = new FixedList128<float>(); // calculated once at startup forever stored with entity
        extents.Add( halfSizeB.x );
        extents.Add( halfSizeB.y );
        extents.Add( halfSizeB.z );

        FixedList128<float3> axisNormals = ColPhysics.GetAxisNormalsOBB( vertices[ 0 ] , vertices[ 1 ] , vertices[ 3 ] , vertices[ 4 ] );

        if ( ColPhysics.SphereIntersectsBox( spherePos , sphereRadius , obbPos , axisNormals , extents , out float distance ) )
        {
            collisionFlag = true;

            if ( resolveCollisions )
            {
                ColPhysics.ResolveSphereOBBCollision( ref spherePos , sphereRadius , ref obbPos , distance );
                posA = spherePos;
                posB = obbPos;
            }
        }
    }

    // CAPSULELOCK CAPSULELOCK
    private void HandleCapsulesLock()
    {
        CollisionCapsuleLock();
        DrawCapsuleLock();
    }
    private void CollisionCapsuleLock()
    {
        float3 p1 = posA;
        float3 p2 = posB;
        float r1 = halfSizeA.x;
        float r2 = halfSizeB.x;
        float h1 = halfSizeA.y*2;
        float h2 = halfSizeB.y*2;

        if ( ColPhysics.ParallelLinesIntersectYAxis( p1.y , p2.y , h1 , h2 ) )
        {
            float3 s1 = new float3( p1.x , 0 , p1.z );
            float3 s2 = new float3( p2.x , 0 , p2.z );

            if ( ColPhysics.SpheresIntersect( s1 , s2 , r1 , r2 , out float distance ) )
            {
                collisionFlag = true;

                if ( resolveCollisions )
                {
                    ColPhysics.ResolveSphereCollision( ref s1 , ref s2 , r1 , r2 , distance );

                    posA = new float3( s1.x , p1.y , s1.z );
                    posB = new float3( s2.x , p2.y , s2.z );
                }
            }
        }
    }

    // CAPSULE CAPSULE
    private void HandleCapsuleCapsule()
    {
        CollisionCapsule();
        DrawCapsules();
    }
    private void CollisionCapsule()
    {
        float3 pa = posA;
        float3 pb = posB;
        float lengthA = halfSizeA.y * 2;
        float lengthB = halfSizeB.y * 2;
        float radiusA = halfSizeA.x;
        float radiusB = halfSizeB.x;

        if ( ColPhysics.CapsuleIntersectsCapsule( pa , pb , rotA , rotB , lengthA , lengthB , radiusA , radiusB , out float distance ) )
        {
            collisionFlag = true;

            if ( resolveCollisions )
            {
                ColPhysics.ResolveSphereCollision( ref pa , ref pb , radiusA , radiusB , distance );

                posA = pa;
                posB = pb;
            }
        }
    }

    // BOX BOX
    private void HandleOBBOBB()
    {
        CollisionOBBOBB();
        DrawBoxes();
    }
    private void CollisionOBBOBB()
    {
        FixedList128<float3> verticesA = ColPhysics.GetAABBVerticesOBB( posA , halfSizeA );
        FixedList128<float3> verticesB = ColPhysics.GetAABBVerticesOBB( posB , halfSizeB );
        verticesA = ColPhysics.GetRotatedVerticesOBB( verticesA , posA , rotA );
        verticesB = ColPhysics.GetRotatedVerticesOBB( verticesB , posB , rotB );
        FixedList512<float3> projectionAxes = ColPhysics.GetProjectionAxesOBBSAT( verticesA , verticesB );

        if ( ColPhysics.BoxesIntersectSAT( projectionAxes , verticesA , verticesB , out float minOverlap , out float3 mtvAxis ) )
        {
            collisionFlag = true;

            if ( resolveCollisions )
            {
                ColPhysics.ResolveBoxCollision( ref posA , ref posB , minOverlap , mtvAxis );
            }
        }
    }

    // POINT TRIANGLE
    public void HandleSphereTriangle()
    {
        float3 spherePos = posA;
        float sphereRadius = halfSizeA.x;
        float3 triPos = posB;
        float3 triSize = halfSizeB;
        float3 triRot = rotB;

        float3[] triangleVerts = GetVerticesOfTriangle( triPos , triSize , triRot ); // would already be stored
        FixedList128<float3> triVertices = new FixedList128<float3>();
        triVertices.Add( triangleVerts[ 0 ] );
        triVertices.Add( triangleVerts[ 1 ] );
        triVertices.Add( triangleVerts[ 2 ] );

        if ( ColPhysics.SphereIntersectsOrientedTriangle( spherePos , sphereRadius , triVertices , out float3 closestPoint , out float distance ) )
        {
            collisionFlag = true;

            if ( resolveCollisions )
            {
                ColPhysics.ResolveSphereTriangleCollision( ref spherePos , sphereRadius , distance , closestPoint );
            }
        }


        /*if ( inTriangle )
        {
            float distance = math.distance( posA , closestPoint );

            if ( distance < halfSizeA.x )
            {
                collisionFlag = true;

                if ( resolveCollisions )
                {
                    float3 direction = math.normalize( posA - closestPoint );
                    float displacement = distance - halfSizeA.x;

                    posA -= direction * displacement;
                }
            }
        }*/

        DrawSphereTriangle();
    }

    // HELPERS
    private float3[] GetVerticesOfTriangle( float3 position , float3 halfSize , float3 rotation )
    {
        float3[] vertices = new float3[ 3 ];
        vertices[ 0 ] = position + new float3( -halfSize.x , -halfSize.y , 0 );
        vertices[ 1 ] = position + new float3( halfSize.x , -halfSize.y , 0 );
        vertices[ 2 ] = position + new float3( 0 , halfSize.y , 0 );

        for ( int i = 0; i < vertices.Length; i++ )
        {
            vertices[ i ] = ColPhysics.RotatePoint3D( vertices[ i ] , position , rotation.y , rotation.z , rotation.x );
        }

        return vertices;
    }

    // DRAW FUNCTIONS
    private void DrawSpheres()
    {
        shapeA.transform.position = posA;
        shapeB.transform.position = posB;

        shapeA.transform.localScale = halfSizeA * 2;
        shapeB.transform.localScale = halfSizeB * 2;
    }
    private void DrawSphereCapsule()
    {
        shapeA.transform.position = posA;
        shapeB.transform.position = posB;

        float xRotation = ColPhysics.DegreesToRadians( 90 );
        shapeB.transform.rotation = quaternion.EulerXYZ( rotB.x + xRotation , rotB.y , rotB.z );

        shapeA.transform.localScale = new float3( halfSizeA.x * 2 , halfSizeA.x * 2 , halfSizeA.x * 2 );
        shapeB.transform.localScale = new float3( halfSizeB.x * 2 , halfSizeB.y * 2 , halfSizeB.x * 2 );
    }
    private void DrawSphereBox()
    {
        shapeA.transform.position = posA;
        shapeB.transform.position = posB;

        shapeB.transform.rotation = quaternion.EulerXYZ( rotB );

        shapeA.transform.localScale = halfSizeA * 2;
        shapeB.transform.localScale = halfSizeB * 2;
    }
    private void DrawCapsuleLock()
    {
        shapeA.transform.position = posA;
        shapeB.transform.position = posB;

        shapeA.transform.localScale = new float3( halfSizeA.x * 2 , halfSizeA.y * 2 , halfSizeA.x * 2 );
        shapeB.transform.localScale = new float3( halfSizeB.x * 2 , halfSizeB.y * 2 , halfSizeB.x * 2 );
    }
    private void DrawCapsules()
    {
        shapeA.transform.position = posA;
        shapeB.transform.position = posB;

        float xRotation = ColPhysics.DegreesToRadians( 90 );
        shapeA.transform.rotation = quaternion.EulerXYZ( rotA.x + xRotation , rotA.y , rotA.z );
        shapeB.transform.rotation = quaternion.EulerXYZ( rotB.x + xRotation , rotB.y , rotB.z );

        shapeA.transform.localScale = new float3( halfSizeA.x * 2 , halfSizeA.y * 2 , halfSizeA.x * 2 );
        shapeB.transform.localScale = new float3( halfSizeB.x * 2 , halfSizeB.y * 2 , halfSizeB.x * 2 );
    }
    private void DrawBoxes()
    {
        shapeA.transform.position = posA;
        shapeB.transform.position = posB;

        shapeA.transform.rotation = quaternion.EulerXYZ( rotA );
        shapeB.transform.rotation = quaternion.EulerXYZ( rotB );

        shapeA.transform.localScale = halfSizeA * 2;
        shapeB.transform.localScale = halfSizeB * 2;
    }
    private void DrawSphereTriangle()
    {
        shapeA.transform.position = posA;
        shapeB.transform.position = posB;
        shapeB.transform.rotation = quaternion.EulerXYZ( new float3( rotB.x , rotB.y , rotB.z ) );
        shapeA.transform.localScale = halfSizeA * 2;

        //closestPointMarker.transform.localScale = new float3( 0.1f , 0.1f , 0.1f );
    }
    private void SetupTriangleMesh()
    {
        Mesh mesh = new Mesh();
        Vector3[] vertices = new Vector3[ 3 ];
        vertices[ 0 ] = posB + new float3( -halfSizeB.x , -halfSizeB.y , 0 );
        vertices[ 1 ] = posB + new float3( 0 , halfSizeB.y , 0 );
        vertices[ 2 ] = posB + new float3( halfSizeB.x , -halfSizeB.y , 0 );
        int[] triangles = new int[ 3 ];
        triangles[ 0 ] = 0;
        triangles[ 1 ] = 1;
        triangles[ 2 ] = 2;
        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();
        shapeB.AddComponent<MeshFilter>();
        shapeB.AddComponent<MeshRenderer>();
        shapeB.GetComponent<MeshFilter>().mesh = mesh;
        Material mat = new Material( Shader.Find( "Standard" ) );
        shapeB.GetComponent<MeshRenderer>().material = mat;
    }
}
