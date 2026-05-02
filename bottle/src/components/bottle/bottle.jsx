import React from 'react';
import './bottle.css'
const bottle = ({bottle_single , handleCartBottle}) => {
// console.log(bottle_single)
const cartFunction =()=>{
    handleCartBottle(bottle_single)
}

    return (
        <div  className='bottle-body'>
            <div className='bottle-details'>
            <div className='grid gap-2'>
            <h3 className='text-2xl font-semibold'>{bottle_single.name}</h3>
            <p>Capacity : {bottle_single.capacity}</p>
            <p>Material : {bottle_single.material}</p>
            <p>Color : {bottle_single.color}</p>
            <p>Price : <span style={{color:'red'}}>{bottle_single.price} $</span></p>
            <h3 className='font-bold'>Brand Name : {bottle_single.brand}</h3>
            <p>Rating : <span style={{color:'goldenrod'}}>{bottle_single.rating}⭐</span></p>
            </div>
            <div>
                <img style={{height:'200px', width:'200px'}} src={bottle_single.image}alt='image loading'></img>
            </div>
            </div>
            <div className='mt-2'><button className='border-2' onClick={cartFunction}>Buy Now</button></div>
        </div>
    );
};

export default bottle;


// {
//     "id": 1,
//     "name": "Water Bottle",
//     "capacity": "500ml",
//     "material": "Plastic",
//     "color": "Blue",
//     "price": 5.99,
//     "isReusable": true,
//     "image": "https://images.pexels.com/photos/346734/pexels-photo-346734.jpeg",
//     "brand": "AquaPure",
//     "rating": 4.5,
//     "description": "A classic and reliable water bottle for everyday hydration."
// }