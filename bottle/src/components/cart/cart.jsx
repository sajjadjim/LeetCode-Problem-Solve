import React from 'react';
import './cart.css'
const Cart = ({cartBottles ,handleCartItems}) => {
    console.log(cartBottles)
    return (
        <div className='cart-container'>
            {
               cartBottles.map(bottle=> <div key={bottle.id}>
                <img   src={bottle.image} alt='' />
                <button onClick={()=>{handleCartItems(bottle.id)}}>x</button>
               </div>)
            }
            
        </div>
    );
};

export default Cart;